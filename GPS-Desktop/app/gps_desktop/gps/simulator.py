"""A simulated GPS node: a car lapping a made-up circuit, sending every message at its real rate.

Frames are built with the same database that decodes them, so the simulator exercises the whole
path a real node would. It also obeys the gate commands and times laps over the gates, so the
gate and lap screens can be tried without hardware.
"""

from __future__ import annotations

import datetime
import math
import random
import threading
import time
from collections import deque
from collections.abc import Iterator

from gps_desktop.gps import enums
from gps_desktop.gps.dbc import COMMAND_MESSAGE, GATE_SET_MESSAGE, Database, Frame
from gps_desktop.gps.sources import Source
from gps_desktop.gps.store import from_enu

TICK = 0.01  # The IMU messages go out at 100 Hz; everything else is a multiple of this.
ORIGIN = (50.93570, -1.39660)
TWO_PI = 2 * math.pi
GATE_HALF_WIDTH_M = 2.0  # LAP_GATE_HALF_WIDTH_M in the firmware
WIND_MPS, WIND_FROM_DEG = 3.0, 240.0


def track_point(a: float) -> tuple[float, float]:
    """East and north, in metres, of the point at angle `a` round the circuit."""
    r = 62.0 * (1 + 0.22 * math.cos(2 * a + 0.6) + 0.10 * math.sin(3 * a))
    return 1.25 * r * math.cos(a), 0.85 * r * math.sin(a)


class Simulator(Source):
    name = "Simulated GPS node"

    def __init__(self, db: Database, lap_seconds: float = 30.0, seed: int | None = None) -> None:
        self.db = db
        self.lap_seconds = lap_seconds
        self.random = random.Random(seed)
        self.commands: deque[Frame] = deque()
        self.angle = 0.0
        self.elapsed = 0.0
        self.counters: dict[str, int] = {}
        self.pace = 1.0  # Changes a little every lap, so lap times differ
        # Gate slot -> (angle round the circuit, or None if off the track, east, north, heading).
        # Heading is an ENU math angle in degrees, as the firmware sends it.
        self.tidy_until = -1.0  # A CONFIG_SAVE shows as pending flash housekeeping until then
        self.gates: dict[int, tuple[float, float, float, float]] = {}
        for slot, angle in enumerate((0.5, 0.5 + TWO_PI / 3, 0.5 + 2 * TWO_PI / 3)):
            self._place_gate(slot, angle)
        self.laps = 0
        self.running = False
        self.lap_start = 0.0
        self.sector_start = 0.0
        self.sector = 0
        self.crossed: set[int] = set()
        self.next_gate = 0
        self.mag_cal_start = 0.0
        self.motion = self._motion(0.0)

    def send(self, frame: Frame) -> bool:
        self.commands.append(frame)
        return True

    def frames(self, stop: threading.Event) -> Iterator[Frame]:
        start = time.monotonic()
        tick = 0
        while not stop.is_set():
            due = start + tick * TICK
            wait = due - time.monotonic()
            if wait > 0 and stop.wait(wait):
                return
            if wait < -1.0:
                # The computer was asleep or busy. Skip ahead rather than send a burst.
                start += -wait
                due = start + tick * TICK
            yield from self.step(tick, due)
            tick += 1

    def step(self, tick: int, t: float) -> list[Frame]:
        """Advance one tick and return the frames due on it, stamped with time `t`."""
        out: list[Frame] = []
        self.elapsed = tick * TICK
        while self.commands:
            self._command(self.commands.popleft())
        previous = self.angle
        self.angle += self._angle_rate(self.angle) * TICK
        self.motion = self._motion(self.angle)
        self._time_laps(previous, out, t)
        m = self.motion
        fix = self.elapsed >= 3.0

        def send(name: str, values: dict[str, float]) -> None:
            if name in self.db.counter_size:
                values["counter"] = self.counters[name] = (self.counters.get(name, -1) + 1) % self.db.counter_size[name]
            out.append(self.db.encode(name, values, t))

        noise = self.random.gauss
        send("GPS_IMU_Accel", {"ax_mg": round(m["ax"] * 1000 + noise(0, 12)),
                               "ay_mg": round(m["ay"] * 1000 + noise(0, 12)),
                               "az_mg": round(1000 + noise(0, 15))})
        send("GPS_IMU_Gyro", {"gx_dps": noise(0, 0.3), "gy_dps": noise(0, 0.3), "gz_dps": m["yaw_rate"] + noise(0, 0.3)})
        if tick % 2 == 0:
            status = 0x03 | (0x04 if self._mag_cal() >= 2 else 0) | (0x38 if fix else 0)
            send("GPS_Attitude", {"yaw_deg": (m["course"] + noise(0, 0.15)) % 360, "pitch_deg": -m["ax"] * 2.5,
                                  "roll_deg": m["ay"] * 3.0, "fusion_status": status})
        if tick % 2 == 1:  # 50 Hz, on the slots the attitude frame does not use, as on the node
            lat, lon = from_enu(m["east"], m["north"], ORIGIN)
            send("GPS_Position", {"lat_deg": lat, "lon_deg": lon})
            send("GPS_Velocity", {"speed_mps": m["speed"], "course_deg": m["course"], "alt_m": 21.4 + noise(0, 0.05),
                                  "fix_type": 3 if fix else 0, "num_sv": min(15, int(4 + self.elapsed * 2))})
        if tick % 10 == 0:
            flags = (enums.LAP_RUNNING if self.running else 0) | (enums.LAP_ARMED if 0 in self.gates else 0)
            running_ms = (t - self.lap_start) * 1000 if self.running else 0
            send("Lap_Status", {"lap": self.laps, "running_time_ms": int(running_ms), "sector": self.sector,
                                "flags": flags | (0 if fix else enums.LAP_DEGRADED)})
            # Airspeed is the ground velocity less a steady wind from the south-west.
            course = math.radians(m["course"])
            wind_to = math.radians(WIND_FROM_DEG + 180)
            air_east = m["speed"] * math.sin(course) - WIND_MPS * math.sin(wind_to)
            air_north = m["speed"] * math.cos(course) - WIND_MPS * math.cos(wind_to)
            air_flags = 0x09 | (0x02 if self.elapsed >= 2 else 0) | (0x04 if self.elapsed >= 20 else 0)
            send("GPS_Airspeed", {"airspeed_mps": math.hypot(air_east, air_north) + abs(noise(0, 0.1)),
                                  "wind_speed_mps": WIND_MPS if air_flags & 0x04 else 0.0,
                                  "wind_from_deg": WIND_FROM_DEG if air_flags & 0x04 else 0.0,
                                  "pitot_scale": 1.04, "air_flags": air_flags})
            yaw = math.radians(m["course"])
            send("GPS_Mag", {"mx_ut": 19.5 * math.cos(yaw) + noise(0, 0.3), "my_ut": -19.5 * math.sin(yaw) + noise(0, 0.3),
                             "mz_ut": -43.2 + noise(0, 0.3), "cal_status": self._mag_cal()})
        if tick % 20 == 0:
            carrier = 0 if self.elapsed < 8 else 1 if self.elapsed < 14 else 2
            send("GPS_Quality", {"hacc_mm": round((1500, 420, 14)[carrier] + abs(noise(0, 2))),
                                 "sacc_mm_s": (300, 90, 25)[carrier], "pdop": 1.2 + abs(noise(0, 0.03)),
                                 "flags": (0x01 if fix else 0) | (carrier << 1) | (0x08 if carrier else 0)})
            slot = self.next_gate
            self.next_gate = (slot + 1) % enums.GATE_COUNT
            _, east, north, heading = self.gates.get(slot, (0.0, 0.0, 0.0, 0.0))
            send("GPS_Gate", {"gate_index": slot, "gate_flags": enums.GATE_VALID if slot in self.gates else 0,
                              "gate_east_m": east, "gate_north_m": north, "gate_heading_deg": heading})
        if tick % 100 == 0:
            now = datetime.datetime.now(datetime.timezone.utc)
            week_ms = ((now.weekday() + 1) % 7 * 86400 + now.hour * 3600 + now.minute * 60 + now.second + 18) * 1000
            send("GPS_Time", {"itow_ms": week_ms % 604800000, "utc_hour": now.hour, "utc_min": now.minute,
                              "utc_sec": now.second, "time_flags": 0x03 if fix else 0})
            send("GPS_Frame_Origin", {"origin_lat_deg": ORIGIN[0], "origin_lon_deg": ORIGIN[1]})
            send("GPS_Temp", {"mcp9800_temp_c": 31.5 + math.sin(self.elapsed / 40), "imu_temp_c": 36.2 + noise(0, 0.05),
                              "mcu_temp_c": 41.0})
            tidy = enums.FLASH_TIDY if self.elapsed < self.tidy_until else 0
            send("GPS_Status", {"uptime_s": min(65535, int(self.elapsed)),
                                "fault_bits": 0x06 | (0x19 if fix else 0) | tidy,
                                "gps_retry_count": 0, "imu_retry_count": 0,
                                "cpu_load_pct": max(0, min(100, round(23 + noise(0, 1.5))))})
        return out

    # The car

    def _angle_rate(self, a: float) -> float:
        """Radians a second round the circuit. Faster on the long sides, slower through the ends."""
        return TWO_PI / self.lap_seconds * self.pace * (1 + 0.3 * math.cos(2 * a + 0.4))

    def _motion(self, a: float) -> dict[str, float]:
        def velocity(angle: float) -> tuple[float, float, float]:
            (e0, n0), (e1, n1) = track_point(angle - 1e-4), track_point(angle + 1e-4)
            rate = self._angle_rate(angle)
            ve, vn = (e1 - e0) / 2e-4 * rate, (n1 - n0) / 2e-4 * rate
            return math.hypot(ve, vn), math.degrees(math.atan2(ve, vn)) % 360, rate

        east, north = track_point(a)
        speed, course, rate = velocity(a)
        ahead_speed, ahead_course, _ = velocity(a + 1e-3)
        dt = 1e-3 / rate
        yaw_rate = ((ahead_course - course + 180) % 360 - 180) / dt  # Degrees a second, clockwise positive
        return {"east": east, "north": north, "speed": speed, "course": course, "yaw_rate": yaw_rate,
                "ax": (ahead_speed - speed) / dt / 9.81, "ay": speed * math.radians(yaw_rate) / 9.81}

    def _mag_cal(self) -> int:
        age = self.elapsed - self.mag_cal_start
        return 0 if age < 2 else 1 if age < 10 else 2 if age < 16 else 3

    # Gates and lap timing

    def _place_gate(self, slot: int, angle: float) -> None:
        east, north = track_point(angle)
        self.gates[slot] = (angle % TWO_PI, east, north, enums.heading_from_bearing(self._motion(angle)["course"]))

    def _set_gate(self, slot: int, place) -> None:
        """Fill a slot the way the firmware does: start/finish clears the sectors and restarts timing."""
        if slot == enums.SLOT_NEXT_SECTOR:
            slot = next((s for s in range(1, enums.GATE_COUNT) if s not in self.gates), enums.GATE_COUNT - 1)
        if not 0 <= slot < enums.GATE_COUNT:
            return
        if slot == 0:
            self.gates.clear()
            self._reset_timing()
        place(slot)

    def _place_gate_at(self, slot: int, east: float, north: float, heading: float) -> None:
        """A gate at a given point. Like the firmware, it only times laps if it lies across the track,
        facing the way the car drives."""
        best = min((math.hypot(e - east, n - north), a)
                   for a in (i * TWO_PI / 3600 for i in range(3600)) for e, n in [track_point(a)])
        distance, angle = best
        travel = math.radians(enums.heading_from_bearing(self._motion(angle)["course"]))
        forward = math.cos(travel - math.radians(heading)) > 0
        self.gates[slot] = (angle if distance <= GATE_HALF_WIDTH_M and forward else None, east, north, heading % 360)

    def _command(self, frame: Frame) -> None:
        decoded = self.db.decode(frame)
        if decoded is None:
            return
        if decoded[0] == GATE_SET_MESSAGE:
            v = decoded[1]
            self._set_gate(int(v["set_index"]), lambda s: self._place_gate_at(
                s, v["set_east_m"], v["set_north_m"], v["set_heading_deg"]))
            return
        if decoded[0] != COMMAND_MESSAGE:
            return
        command, slot = int(decoded[1]["cmd"]), int(decoded[1]["arg0"])
        if command == enums.CMD_GATE_SET:
            self._set_gate(slot, lambda s: self._place_gate(s, self.angle))
        elif command == enums.CMD_GATE_CLEAR:
            for s in range(enums.GATE_COUNT) if slot == enums.SLOT_ALL else [slot]:
                self.gates.pop(s, None)
            if 0 not in self.gates:
                self._reset_timing()
        elif command == enums.CMD_MAG_CAL_START:
            self.mag_cal_start = self.elapsed
        elif command == enums.CMD_CONFIG_SAVE:
            self.tidy_until = self.elapsed + 3.0

    def _reset_timing(self) -> None:
        self.laps = 0
        self.running = False
        self.sector = 0
        self.crossed.clear()

    def _time_laps(self, previous: float, out: list[Frame], t: float) -> None:
        def event(kind: int, lap: int, time_ms: float) -> None:
            self.counters["Lap_Event"] = (self.counters.get("Lap_Event", -1) + 1) % self.db.counter_size["Lap_Event"]
            out.append(self.db.encode("Lap_Event", {"type": kind, "lap": lap, "time_ms": int(round(time_ms)),
                                                    "counter": self.counters["Lap_Event"]}, t))

        for slot, (gate_angle, *_rest) in sorted(self.gates.items()):
            if gate_angle is None:
                continue  # Not across the track, so never crossed
            before = (previous - gate_angle) % TWO_PI
            after = (self.angle - gate_angle) % TWO_PI
            if after >= before:
                continue  # Not crossed on this tick.
            # Where in the tick the car crossed, so times are finer than the tick itself.
            crossing = t - TICK * after / (after + TWO_PI - before)
            if slot == 0:
                if self.running:
                    if self.sector > 0:
                        event(enums.EVENT_SECTOR, self.laps, (crossing - self.sector_start) * 1000)
                    self.laps += 1
                    event(enums.EVENT_LAP, self.laps, (crossing - self.lap_start) * 1000)
                self.running = True
                self.pace = self.random.uniform(0.96, 1.04)
                self.lap_start = self.sector_start = crossing
                self.sector = 0
                self.crossed.clear()
            elif self.running and slot not in self.crossed:
                event(enums.EVENT_SECTOR, self.laps, (crossing - self.sector_start) * 1000)
                self.sector_start = crossing
                self.sector += 1
                self.crossed.add(slot)
