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
import queue
import threading
from collections.abc import Iterator

from gps_dashboard.can_sources.base import CanSource
from gps_dashboard.decoder import Decoder
from gps_dashboard.raw_frame import RawFrame

#: GPS_Command sub-commands the simulator honours (see can_defs.h). Lets a
#: dashboard control panel drive the simulated node exactly as the
#: steering-wheel buttons drive the real one.
_CMD_GATE_SET = 0x01
_CMD_GATE_CLEAR = 0x02
_GATE_CLEAR_ALL = 0xFF

#: Lap_Event.type values (see CAN_LAP_EVENT_* in can_defs.h).
_LAP_EVENT_LAP = 0
_LAP_EVENT_SECTOR = 1

#: Gate slots (start/finish + 7 sectors), matching LAP_MAX_GATES.
_MAX_GATE_SLOTS = 8

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
#: A closed loop with a genuine main straight along the bottom (the
#: start/finish sits on it, not in a corner), then a fast right-hand
#: sweeper, a tighter left complex up top, and back down the left side.
#: The three collinear points along the bottom keep that section dead
#: straight through the Catmull-Rom spline; kept within ~+-36 m east so it
#: stays close to the origin.
_TRACK_WAYPOINTS: list[tuple[float, float]] = [
    (-18.0, 0.0),   # start of the main straight
    (0.0, 0.0),     # start/finish line, mid-straight, heading east
    (18.0, 0.0),    # end of the main straight
    (32.0, 8.0),    # turn 1, into the right sweeper
    (37.0, 26.0),
    (28.0, 42.0),   # top-right
    (12.0, 48.0),
    (2.0, 40.0),    # tighter left complex
    (8.0, 28.0),
    (-4.0, 24.0),
    (-20.0, 30.0),  # top-left sweeper
    (-33.0, 40.0),
    (-38.0, 22.0),  # last corner
    (-32.0, 6.0),   # onto the main straight
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

#: Speed-profile shape limits (m/s^2). These set the *relative* corner-vs-
#: straight speeds and how sharply the car brakes/accelerates between them;
#: the whole profile is then time-scaled so a lap takes lap_period_s, so
#: their absolute values don't fix the absolute speed - only its shape.
_A_LAT_MAX = 12.0    # cornering grip -> corner speed = sqrt(a_lat/curvature)
_A_ACCEL_MAX = 6.0   # power-limited acceleration out of a corner
_A_BRAKE_MAX = 11.0  # braking into a corner
_V_STRAIGHT_MAX = 45.0  # top speed cap on a straight (pre-scaling)


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
        self._build_speed_profile()

        # Gate slots (0 = start/finish, 1..7 = sectors). Each keeps its
        # arc-length position `s` for crossing detection plus the ENU
        # position/heading for the broadcast. Pre-seeded with a default
        # layout so the demo has a track immediately; the control panel can
        # then set/clear them at runtime (mirroring the steering wheel).
        self._gates: list[dict[str, float]] = [
            {"s": 0.0, "east_m": 0.0, "north_m": 0.0, "heading_deg": 0.0,
             "valid": 0.0}
            for _ in range(_MAX_GATE_SLOTS)
        ]
        for frac, index in _GATE_FRACTIONS:
            self._place_gate(index, frac * self._length_m)

        # GPS_Commands arrive from another thread (the GUI); frames()
        # drains and applies them in its own thread, so gate/engine state
        # is only ever touched from one thread.
        self._cmd_queue: queue.Queue[RawFrame] = queue.Queue()

    def _place_gate(self, slot: int, s: float) -> None:
        """Set gate `slot` at arc-length `s` on the path, filling its ENU
        position/heading from the path there."""
        e, n, heading_rad, _, _, _ = self._sample(s)
        self._gates[slot] = {
            "s": s % self._length_m,
            "east_m": e,
            "north_m": n,
            "heading_deg": math.degrees(heading_rad) % 360.0,
            "valid": 1.0,
        }

    def send(self, frame: RawFrame) -> None:
        """Queue a GPS_Command frame for the simulated node to act on."""
        self._cmd_queue.put(frame)

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

    def _build_speed_profile(self) -> None:
        """A per-point target speed: cornering-grip-limited where the path
        is curved, then forward/backward passes so the car brakes into and
        accelerates out of corners (not step changes), and finally
        time-scaled so one lap takes lap_period_s. Also stores the
        longitudinal acceleration that profile implies."""
        cum = self._cum
        u = len(self._pts) - 1  # unique points (last duplicates the first)
        ds = [cum[i + 1] - cum[i] for i in range(u)]

        # Grip-limited corner speed: v = sqrt(a_lat / curvature).
        v = []
        for i in range(u):
            k = abs(self._kappa[i])
            v.append(
                _V_STRAIGHT_MAX
                if k < 1e-6
                else min(_V_STRAIGHT_MAX, math.sqrt(_A_LAT_MAX / k))
            )

        # Two wrap-around passes converge the loop seam: brake (backward)
        # then accelerate (forward), each capping the speed a point can
        # hold given its neighbour and the accel/brake limit over ds.
        for _ in range(2):
            for i in range(u - 1, -1, -1):
                nxt = (i + 1) % u
                v[i] = min(v[i], math.sqrt(v[nxt] ** 2 + 2.0 * _A_BRAKE_MAX * ds[i]))
            for i in range(u):
                prv = (i - 1) % u
                v[i] = min(
                    v[i], math.sqrt(v[prv] ** 2 + 2.0 * _A_ACCEL_MAX * ds[prv])
                )

        # Time-scale so the lap takes lap_period_s (scaling every speed by
        # m divides lap time by m).
        lap_time = sum(ds[i] / v[i] for i in range(u))
        m = lap_time / self._lap_period_s
        v = [x * m for x in v]

        # Longitudinal accel a = v dv/ds (>0 accelerating, <0 braking).
        along = []
        for i in range(u):
            nxt = (i + 1) % u
            along.append(v[i] * (v[nxt] - v[i]) / ds[i] if ds[i] > 1e-9 else 0.0)

        v.append(v[0])
        along.append(along[0])
        self._speed = v
        self._along = along

    def _sample(
        self, dist_m: float
    ) -> tuple[float, float, float, float, float, float]:
        """(east, north, heading_rad, signed_curvature, speed_mps,
        longitudinal_accel) at arc-length dist_m along the loop (wrapped)."""
        d = dist_m % self._length_m
        i = bisect.bisect_right(self._cum, d) - 1
        i = max(0, min(i, len(self._pts) - 2))
        seg = self._cum[i + 1] - self._cum[i]
        frac = (d - self._cum[i]) / seg if seg > 1e-9 else 0.0
        e = self._pts[i][0] + frac * (self._pts[i + 1][0] - self._pts[i][0])
        n = self._pts[i][1] + frac * (self._pts[i + 1][1] - self._pts[i][1])
        return e, n, self._heading[i], self._kappa[i], self._speed[i], self._along[i]

    # -- gate commands + lap-timing engine ---------------------------------

    def _apply_commands(self, dist_m: float, t: float) -> None:
        """Drain queued GPS_Commands and apply them to the gate table /
        timing engine, so the control panel can plant/clear gates exactly
        like the steering-wheel buttons. Gates are placed at the car's
        current position (arc-length dist_m), same as the real node uses
        its current fused position."""
        while True:
            try:
                frame = self._cmd_queue.get_nowait()
            except queue.Empty:
                break
            decoded = self._decoder.decode(frame)
            if decoded is None or decoded.name != "GPS_Command":
                continue
            cmd = int(decoded.signals["cmd"])
            arg0 = int(decoded.signals["arg0"])

            if cmd == _CMD_GATE_SET and arg0 < _MAX_GATE_SLOTS:
                self._place_gate(arg0, dist_m)
                if arg0 == 0:
                    # A new start/finish wipes the sectors and restarts the
                    # timing reference here (mirrors gates.c + a clean lap).
                    for i in range(1, _MAX_GATE_SLOTS):
                        self._gates[i]["valid"] = 0.0
                    self._running = True
                    self._lap_start_t = t
                    self._sector_start_t = t
                    self._current_sector = 0
            elif cmd == _CMD_GATE_CLEAR:
                if arg0 == _GATE_CLEAR_ALL:
                    for g in self._gates:
                        g["valid"] = 0.0
                    self._running = False
                elif arg0 < _MAX_GATE_SLOTS:
                    self._gates[arg0]["valid"] = 0.0

    @staticmethod
    def _crossed(prev_d: float, cur_d: float, s: float, length: float) -> bool:
        """True if marching from prev_d to cur_d passed gate arc-length s
        (works across the lap seam; at most one crossing per tick)."""
        return math.floor((cur_d - s) / length) > math.floor(
            (prev_d - s) / length
        )

    def _update_timing(
        self, prev_d: float, cur_d: float, t: float
    ) -> list[tuple[int, int, int]]:
        """Detect gate crossings between prev_d and cur_d and advance the
        lap/sector timer. Returns (type, lap, time_ms) tuples for any
        Lap_Events to broadcast this tick."""
        events: list[tuple[int, int, int]] = []
        if cur_d <= prev_d:
            return events
        length = self._length_m

        # Sectors first: a start/finish crossing in the same tick then
        # resets the sector counter below.
        for slot in range(1, _MAX_GATE_SLOTS):
            g = self._gates[slot]
            if (
                g["valid"]
                and self._running
                and self._crossed(prev_d, cur_d, g["s"], length)
            ):
                sector_ms = int((t - self._sector_start_t) * 1000.0)
                self._sector_start_t = t
                self._current_sector += 1
                events.append((_LAP_EVENT_SECTOR, self._lap_count, sector_ms))

        g0 = self._gates[0]
        if g0["valid"] and self._crossed(prev_d, cur_d, g0["s"], length):
            if self._running:
                lap_ms = int((t - self._lap_start_t) * 1000.0)
                self._lap_count += 1
                events.append((_LAP_EVENT_LAP, self._lap_count, lap_ms))
            self._running = True
            self._lap_start_t = t
            self._sector_start_t = t
            self._current_sector = 0

        return events

    def frames(self, stop: threading.Event) -> Iterator[RawFrame]:
        t = 0.0
        counters = {name: 0 for name, _ in _MESSAGE_RATES}
        lap_event_counter = 0
        gate_rr = 0  # round-robin index for the GPS_Gate broadcast
        # Per-message "next due" time, staggered slightly so not every
        # message fires on the very first tick at once.
        next_due = {
            name: (i / len(_MESSAGE_RATES)) / rate
            for i, (name, rate) in enumerate(_MESSAGE_RATES)
        }

        # Lap-timing engine state. Seeded "running" from the start line at
        # t=0 (the car begins on the default start/finish), so the first
        # completed lap reports a real time after one lap rather than two.
        self._running = True
        self._lap_count = 0
        self._current_sector = 0
        self._lap_start_t = 0.0
        self._sector_start_t = 0.0

        # March by arc length at the profiled (variable) speed, so the car
        # slows for corners and accelerates on the straights.
        dist_m = 0.0
        prev_dist = 0.0

        while not stop.is_set():
            east_m, north_m, heading_rad, kappa, speed_mps, along_mps2 = (
                self._sample(dist_m)
            )
            lat_deg = self._origin_lat + north_m / _M_PER_DEG_LAT
            lon_deg = self._origin_lon + east_m / self._m_per_deg_lon

            # Path tangent is an ENU math bearing (0 = east, CCW); the
            # dashboard's compass heading is 0 = north, clockwise, i.e.
            # (90 - math_bearing) mod 360.
            heading_deg = (90.0 - math.degrees(heading_rad)) % 360.0

            # Lateral load and yaw rate come straight from path curvature at
            # the current speed; longitudinal load from the speed profile.
            # Signed, so the dashboard shows left/right and accel/brake.
            lateral_g = (speed_mps * speed_mps * kappa) / 9.81
            longitudinal_g = along_mps2 / 9.81
            yaw_rate_dps = math.degrees(speed_mps * kappa)

            # Detect gate crossings (against the gates as they stand this
            # tick), then apply any control-panel commands so a just-placed
            # gate isn't instantly "crossed".
            lap_events = self._update_timing(prev_dist, dist_m, t)
            self._apply_commands(dist_m, t)

            running_time_ms = (
                int((t - self._lap_start_t) * 1000.0) if self._running else 0
            )

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
                    # Round-robin one slot per fire, over all 8 slots so the
                    # dash learns which are empty (valid=0) too.
                    g = self._gates[gate_rr]
                    slot = gate_rr
                    gate_rr = (gate_rr + 1) % _MAX_GATE_SLOTS
                    signals = {
                        "gate_index": slot,
                        "gate_flags": 0x01 if g["valid"] else 0x00,
                        "gate_east_m": g["east_m"],
                        "gate_north_m": g["north_m"],
                        "gate_heading_deg": g["heading_deg"],
                    }
                elif name == "Lap_Status":
                    signals = {
                        "lap": self._lap_count,
                        "running_time_ms": running_time_ms,
                        "sector": self._current_sector,
                        "flags": 0x01 if self._running else 0x00,
                    }
                else:
                    signals = self._build_signals(
                        name,
                        lat_deg=lat_deg,
                        lon_deg=lon_deg,
                        speed_mps=speed_mps,
                        heading_deg=heading_deg,
                        lateral_g=lateral_g,
                        longitudinal_g=longitudinal_g,
                        yaw_rate_dps=yaw_rate_dps,
                        counter=counters[name],
                    )
                yield self._decoder.encode(name, signals)

            # Lap/sector events fire on crossing, not on a fixed rate.
            for ev_type, ev_lap, ev_ms in lap_events:
                lap_event_counter = (lap_event_counter + 1) % 256
                yield self._decoder.encode(
                    "Lap_Event",
                    {
                        "type": ev_type,
                        "lap": ev_lap,
                        "time_ms": ev_ms,
                        "counter": lap_event_counter,
                    },
                )

            prev_dist = dist_m
            dist_m += speed_mps * _TICK_DT
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
        longitudinal_g: float,
        yaw_rate_dps: float,
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
        if name == "GPS_Quality":
            return {"hacc_mm": 15.0, "sacc_mm_s": 20.0, "pdop": 1.2, "flags": 0x01}
        if name == "GPS_IMU_Accel":
            return {
                "ax_mg": _clamp(longitudinal_g * 1000.0, _MAX_ACCEL_MG),
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
