"""Synthetic GPS/IMU traffic for developing and demoing the dashboard
without any CAN hardware attached.

Drives a simulated car around a closed racing circuit (straights, a
hairpin, a chicane and some sweepers - not just a circle) and encodes
every GPS.dbc message at its real nominal rate (see board_config.h / the
plan's CAN matrix), through the *same* `Decoder.encode()` a real
bench-side GPS.dbc round trip would use - so the whole ingest/decode
pipeline is exercised, not bypassed.

The circuit is a Catmull-Rom spline through a handful of waypoints, so
heading, lateral acceleration and yaw rate all fall out of the real path
geometry (curvature) rather than being faked - corners load up the
lateral g and yaw rate, straights don't.
"""

from __future__ import annotations

import bisect
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
    ("GPS_Frame_Origin", 1.0),
    # One slot per fire, round-robined over the gate set (matches the
    # firmware's ~5 Hz round-robin in canbc_task.c).
    ("GPS_Gate", 5.0),
]

#: Circuit shape: waypoints (east, north) in metres for a nominal
#: track_radius_m=30 track, scaled by track_radius_m/30 at construction.
#: A closed loop with a start straight, a tight left hairpin, a chicane
#: and a fast right sweeper - kept within ~+-36 m east so it stays close
#: to the origin. The Catmull-Rom spline below rounds the corners.
_TRACK_WAYPOINTS: list[tuple[float, float]] = [
    (0.0, 0.0),     # start/finish, on the start straight
    (0.0, 30.0),    # start straight, heading north
    (-6.0, 46.0),   # turn 1 (left kink)
    (-22.0, 52.0),
    (-32.0, 40.0),  # left hairpin
    (-26.0, 24.0),
    (-12.0, 22.0),
    (-18.0, 9.0),   # chicane
    (-4.0, 3.0),
    (12.0, 7.0),
    (30.0, 5.0),    # right sweeper
    (36.0, 22.0),
    (26.0, 38.0),
    (12.0, 40.0),
    (5.0, 22.0),    # back onto the start straight
]

#: Gate slots as fractions of a lap (arc length): start/finish plus three
#: sector splits, evenly spaced. Placed on the spline so the dashboard's
#: gate lines land on the trail.
_GATE_FRACTIONS: list[tuple[float, int]] = [
    (0.0, 0),
    (0.25, 1),
    (0.5, 2),
    (0.75, 3),
]

#: Fine samples per waypoint segment when flattening the spline to a
#: polyline for arc-length marching.
_SAMPLES_PER_SEGMENT = 40


def _catmull_rom(
    p0: tuple[float, float],
    p1: tuple[float, float],
    p2: tuple[float, float],
    p3: tuple[float, float],
    t: float,
) -> tuple[float, float]:
    """Point at parameter t in [0,1] on the Catmull-Rom segment from p1 to
    p2 (p0/p3 are the neighbouring control points)."""
    t2 = t * t
    t3 = t2 * t
    out = []
    for a, b, c, d in zip(p0, p1, p2, p3):
        out.append(
            0.5
            * (
                (2.0 * b)
                + (-a + c) * t
                + (2.0 * a - 5.0 * b + 4.0 * c - d) * t2
                + (-a + 3.0 * b - 3.0 * c + d) * t3
            )
        )
    return out[0], out[1]


def _wrap_pi(x: float) -> float:
    while x > math.pi:
        x -= 2.0 * math.pi
    while x <= -math.pi:
        x += 2.0 * math.pi
    return x

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

        # Flatten the closed Catmull-Rom circuit (scaled by radius) into a
        # fine polyline, and precompute per-point arc length, heading (ENU
        # math bearing, 0 = east CCW) and signed curvature. The car marches
        # along this by arc length; heading/curvature drive the attitude,
        # lateral-g and yaw-rate signals.
        scale = track_radius_m / 30.0
        wps = [(e * scale, n * scale) for e, n in _TRACK_WAYPOINTS]
        self._build_path(wps)

        # Gates sit on the path at fixed lap fractions, taking the path's
        # own heading there - so their lines lie across the trail.
        self._gates: list[dict[str, float | int]] = []
        for frac, index in _GATE_FRACTIONS:
            e, n, heading_rad, _ = self._state_at(frac * self._length_m)
            self._gates.append(
                {
                    "index": index,
                    "east_m": e,
                    "north_m": n,
                    "heading_deg": math.degrees(heading_rad) % 360.0,
                }
            )

    def _build_path(self, wps: list[tuple[float, float]]) -> None:
        n = len(wps)
        pts: list[tuple[float, float]] = []
        for i in range(n):
            p0 = wps[(i - 1) % n]
            p1 = wps[i]
            p2 = wps[(i + 1) % n]
            p3 = wps[(i + 2) % n]
            for s in range(_SAMPLES_PER_SEGMENT):
                pts.append(_catmull_rom(p0, p1, p2, p3, s / _SAMPLES_PER_SEGMENT))
        pts.append(pts[0])  # close the loop

        cum = [0.0]
        heading = []
        for i in range(len(pts) - 1):
            de = pts[i + 1][0] - pts[i][0]
            dn = pts[i + 1][1] - pts[i][1]
            seg = math.hypot(de, dn)
            cum.append(cum[-1] + seg)
            heading.append(math.atan2(dn, de))
        heading.append(heading[-1])

        # Signed curvature per point: heading change over the segment.
        kappa = [0.0] * len(pts)
        for i in range(1, len(pts) - 1):
            ds = cum[i + 1] - cum[i]
            if ds > 1e-9:
                kappa[i] = _wrap_pi(heading[i] - heading[i - 1]) / ds

        self._pts = pts
        self._cum = cum
        self._heading = heading
        self._kappa = kappa
        self._length_m = cum[-1]

    def _state_at(self, dist_m: float) -> tuple[float, float, float, float]:
        """(east, north, heading_rad, signed_curvature) at arc-length
        dist_m along the closed loop (wrapped)."""
        d = dist_m % self._length_m
        i = bisect.bisect_right(self._cum, d) - 1
        i = max(0, min(i, len(self._pts) - 2))
        seg = self._cum[i + 1] - self._cum[i]
        frac = (d - self._cum[i]) / seg if seg > 1e-9 else 0.0
        e = self._pts[i][0] + frac * (self._pts[i + 1][0] - self._pts[i][0])
        n = self._pts[i][1] + frac * (self._pts[i + 1][1] - self._pts[i][1])
        return e, n, self._heading[i], self._kappa[i]

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

        # Constant-speed march so a lap takes lap_period_s regardless of
        # the (fixed) circuit length.
        speed_mps = self._length_m / self._lap_period_s

        while not stop.is_set():
            east_m, north_m, heading_rad, kappa = self._state_at(speed_mps * t)
            lat_deg = self._origin_lat + north_m / _M_PER_DEG_LAT
            lon_deg = self._origin_lon + east_m / self._m_per_deg_lon

            # Path tangent is an ENU math bearing (0 = east, CCW); the
            # dashboard's compass heading is 0 = north, clockwise, i.e.
            # (90 - math_bearing) mod 360.
            heading_deg = (90.0 - math.degrees(heading_rad)) % 360.0

            # Lateral load and yaw rate come straight from path curvature:
            # a_lat = v^2 * kappa, yaw_rate = v * kappa. Signed, so the
            # dashboard shows left/right corners correctly.
            lateral_g = (speed_mps * speed_mps * kappa) / 9.81
            yaw_rate_dps = math.degrees(speed_mps * kappa)
            omega = speed_mps * kappa  # kept for GPS_IMU_Gyro below

            if t - lap_start_t >= self._lap_period_s:
                lap += 1
                lap_start_t = t
            running_time_ms = int((t - lap_start_t) * 1000.0)

            for name, rate in _MESSAGE_RATES:
                if t + 1e-9 < next_due[name]:
                    continue
                next_due[name] += 1.0 / rate
                counters[name] = (counters[name] + 1) % 256

                if name == "GPS_Frame_Origin":
                    signals = {
                        "origin_lat_deg": self._origin_lat,
                        "origin_lon_deg": self._origin_lon,
                    }
                elif name == "GPS_Gate":
                    # Round-robin one gate slot per fire.
                    g = self._gates[counters[name] % len(self._gates)]
                    signals = {
                        "gate_index": g["index"],
                        "gate_flags": 0x01,  # bit0 = valid
                        "gate_east_m": g["east_m"],
                        "gate_north_m": g["north_m"],
                        "gate_heading_deg": g["heading_deg"],
                    }
                else:
                    signals = self._build_signals(
                        name,
                        lat_deg=lat_deg,
                        lon_deg=lon_deg,
                        speed_mps=speed_mps,
                        heading_deg=heading_deg,
                        lateral_g=lateral_g,
                        yaw_rate_dps=yaw_rate_dps,
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
        lateral_g: float,
        yaw_rate_dps: float,
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
                "ay_mg": _clamp(lateral_g * 1000.0, _MAX_ACCEL_MG),
                "az_mg": _clamp(1000.0, _MAX_ACCEL_MG),
                "counter": counter,
            }
        if name == "GPS_IMU_Gyro":
            return {
                "gx_dps": 0.0,
                "gy_dps": 0.0,
                "gz_dps": _clamp(yaw_rate_dps, _MAX_GYRO_DPS),
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
