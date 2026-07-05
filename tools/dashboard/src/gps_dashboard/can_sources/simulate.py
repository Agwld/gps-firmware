"""Synthetic GPS/IMU traffic for developing and demoing the dashboard
without any CAN hardware attached.

Drives a simulated car around a circular track and encodes every
GPS.dbc message at its real nominal rate (see board_config.h / the plan's
CAN matrix), through the *same* `Decoder.encode()` a real bench-side
GPS.dbc round trip would use - so the whole ingest/decode pipeline is
exercised, not bypassed.
"""

from __future__ import annotations

import math
import threading
from collections.abc import Iterator

from gps_dashboard.can_sources.base import CanSource
from gps_dashboard.decoder import Decoder
from gps_dashboard.raw_frame import RawFrame

#: Metres per degree of latitude, and (at the origin latitude) of
#: longitude - the same flat-earth approximation the firmware's own
#: geodesy.c uses for its local ENU frame; plenty accurate for a
#: circular test track a few tens of metres across.
_M_PER_DEG_LAT = 111_320.0

#: Roughly Southampton (Boldrewood Campus), for a homely default origin.
DEFAULT_ORIGIN_LAT_DEG = 50.9368
DEFAULT_ORIGIN_LON_DEG = -1.4045

#: (name, rate_hz) for every message this simulator drives. Rates match
#: the CAN matrix in the design plan.
_MESSAGE_RATES: list[tuple[str, float]] = [
    ("GPS_Position", 20.0),
    ("GPS_Velocity", 20.0),
    ("GPS_Attitude", 50.0),
    ("Lap_Status", 10.0),
    ("GPS_Quality", 5.0),
    ("GPS_IMU_Accel", 100.0),
    ("GPS_IMU_Gyro", 100.0),
    ("GPS_Temp", 1.0),
    ("GPS_Status", 1.0),
    ("GPS_Mag", 10.0),
]

#: Base simulation tick - the fastest message rate above, so every
#: message fires on some tick boundary without needing its own timer.
_TICK_HZ = 100.0
_TICK_DT = 1.0 / _TICK_HZ

#: Real hardware limits (can_defs.h / board_config.h: LSM6DSO32 configured
#: for +-16 g / +-2000 dps). Extreme track_radius_m/lap_period_s
#: combinations can otherwise derive a centripetal acceleration cantools
#: can't encode into GPS_IMU_Accel's i16 range at all, which would crash
#: this generator instead of just producing an (admittedly unrealistic)
#: clamped reading - so clamp before encoding rather than trust the
#: caller's track parameters to always be physically sane.
_MAX_ACCEL_MG = 16_000.0
_MAX_GYRO_DPS = 2000.0


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


class SimulateCanSource(CanSource):
    def __init__(
        self,
        decoder: Decoder,
        track_radius_m: float = 30.0,
        lap_period_s: float = 20.0,
        origin_lat_deg: float = DEFAULT_ORIGIN_LAT_DEG,
        origin_lon_deg: float = DEFAULT_ORIGIN_LON_DEG,
    ) -> None:
        self._decoder = decoder
        self._radius_m = track_radius_m
        self._lap_period_s = lap_period_s
        self._origin_lat = origin_lat_deg
        self._origin_lon = origin_lon_deg
        self._m_per_deg_lon = _M_PER_DEG_LAT * math.cos(math.radians(origin_lat_deg))

    def frames(self, stop: threading.Event) -> Iterator[RawFrame]:
        t = 0.0
        lap = 0
        lap_start_t = 0.0
        counters = {name: 0 for name, _ in _MESSAGE_RATES}
        # Per-message "next due" time, staggered slightly so not every
        # message fires on the very first tick at once.
        next_due = {
            name: (i / len(_MESSAGE_RATES)) / rate
            for i, (name, rate) in enumerate(_MESSAGE_RATES)
        }

        while not stop.is_set():
            omega = 2.0 * math.pi / self._lap_period_s
            angle = omega * t
            speed_mps = (2.0 * math.pi * self._radius_m) / self._lap_period_s

            east_m = self._radius_m * math.sin(angle)
            north_m = self._radius_m * (1.0 - math.cos(angle))
            lat_deg = self._origin_lat + north_m / _M_PER_DEG_LAT
            lon_deg = self._origin_lon + east_m / self._m_per_deg_lon

            # `angle` is the car's position around the circle's centre
            # (0 deg = east of centre, increasing counterclockwise in the
            # east-right/north-up plane below) - NOT the compass bearing
            # of its direction of travel, which is what heading_deg must
            # be (0 = north, 90 = east, clockwise, matching GPS_Attitude
            # and track_map.py's arrow math). Differentiating the position
            # gives a velocity direction of (cos(angle), sin(angle)),
            # whose compass bearing is (90 - angle) mod 360 - using
            # `angle` directly here pointed the heading roughly opposite
            # the actual direction of travel.
            heading_deg = (90.0 - math.degrees(angle)) % 360.0
            centripetal_g = (speed_mps * speed_mps) / (self._radius_m * 9.81)

            if t - lap_start_t >= self._lap_period_s:
                lap += 1
                lap_start_t = t
            running_time_ms = int((t - lap_start_t) * 1000.0)

            for name, rate in _MESSAGE_RATES:
                if t + 1e-9 < next_due[name]:
                    continue
                next_due[name] += 1.0 / rate
                counters[name] = (counters[name] + 1) % 256
                signals = self._build_signals(
                    name,
                    lat_deg=lat_deg,
                    lon_deg=lon_deg,
                    speed_mps=speed_mps,
                    heading_deg=heading_deg,
                    centripetal_g=centripetal_g,
                    omega=omega,
                    lap=lap,
                    running_time_ms=running_time_ms,
                    counter=counters[name],
                )
                yield self._decoder.encode(name, signals)

            t += _TICK_DT
            stop.wait(_TICK_DT)

    @staticmethod
    def _build_signals(
        name: str,
        *,
        lat_deg: float,
        lon_deg: float,
        speed_mps: float,
        heading_deg: float,
        centripetal_g: float,
        omega: float,
        lap: int,
        running_time_ms: int,
        counter: int,
    ) -> dict[str, float | int]:
        if name == "GPS_Position":
            return {"lat_deg": lat_deg, "lon_deg": lon_deg}
        if name == "GPS_Velocity":
            return {
                "speed_mps": speed_mps,
                "course_deg": heading_deg,
                "alt_m": 30.0,
                "fix_type": 3,
                "num_sv": 14,
                "counter": counter,
            }
        if name == "GPS_Attitude":
            return {
                "yaw_deg": heading_deg,
                "pitch_deg": 0.0,
                "roll_deg": 0.0,
                "fusion_status": 0,
                "counter": counter,
            }
        if name == "Lap_Status":
            return {
                "lap": lap,
                "running_time_ms": running_time_ms,
                "sector": 0,
                "flags": 0x01,  # bit0: running
            }
        if name == "GPS_Quality":
            return {"hacc_mm": 15.0, "sacc_mm_s": 20.0, "pdop": 1.2, "flags": 0x01}
        if name == "GPS_IMU_Accel":
            return {
                "ax_mg": 0.0,
                "ay_mg": _clamp(centripetal_g * 1000.0, _MAX_ACCEL_MG),
                "az_mg": _clamp(1000.0, _MAX_ACCEL_MG),
                "counter": counter,
            }
        if name == "GPS_IMU_Gyro":
            return {
                "gx_dps": 0.0,
                "gy_dps": 0.0,
                "gz_dps": _clamp(math.degrees(omega), _MAX_GYRO_DPS),
                "counter": counter,
            }
        if name == "GPS_Temp":
            return {"mcp9800_temp_c": 32.0, "imu_temp_c": 38.0, "mcu_temp_c": 41.0}
        if name == "GPS_Status":
            return {
                "uptime_s": int(counter),
                "fault_bits": 0x0007,  # GPS_READY|IMU_READY|MAG_READY
                "gps_retry_count": 0,
                "imu_retry_count": 0,
                "cpu_load_pct": 18,
            }
        if name == "GPS_Mag":
            return {
                "mx_ut": 20.0 * math.cos(math.radians(heading_deg)),
                "my_ut": 20.0 * math.sin(math.radians(heading_deg)),
                "mz_ut": -40.0,
                "cal_status": 2,
            }
        raise ValueError(f"simulator has no generator for message {name!r}")
