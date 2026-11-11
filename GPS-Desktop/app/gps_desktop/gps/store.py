"""The live data store: latest value, update time and recent history of every signal, plus the
track, gates and lap times built up from them.

Whatever is connected pushes decoded messages in with ingest(). Screens only read from here.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field

import numpy as np
from PySide6.QtCore import QObject, QTimer, Signal

from gps_desktop.gps import enums
from gps_desktop.gps.dbc import Database

EARTH_RADIUS_M = 6378137.0


class History:
    """A fixed-size ring buffer of (time, value) samples for one signal."""

    def __init__(self, size: int) -> None:
        self.t = np.zeros(size)
        self.v = np.zeros(size)
        self.size = size
        self.count = 0
        self.head = 0

    def append(self, t: float, v: float) -> None:
        self.t[self.head] = t
        self.v[self.head] = v
        self.head = (self.head + 1) % self.size
        self.count = min(self.count + 1, self.size)

    def arrays(self) -> tuple[np.ndarray, np.ndarray]:
        """Samples in time order, oldest first."""
        if self.count < self.size:
            return self.t[: self.count].copy(), self.v[: self.count].copy()
        return np.roll(self.t, -self.head), np.roll(self.v, -self.head)

    def clear(self) -> None:
        self.count = 0
        self.head = 0


@dataclass
class Gate:
    index: int
    east: float  # Metres from the node's frame origin
    north: float
    heading: float  # Direction of travel, degrees anticlockwise from east (as on the bus)


@dataclass
class Lap:
    number: int
    time_ms: int | None = None  # None while the lap is still being driven
    sectors: list[int] = field(default_factory=list)


@dataclass(frozen=True)
class Event:
    wall: float  # time.time() when it arrived
    kind: int  # enums.EVENT_*
    lap: int
    time_ms: int


def to_enu(lat, lon, origin: tuple[float, float]):
    """Metres east and north of the origin. Accurate to millimetres over the size of a circuit."""
    lat0, lon0 = origin
    east = np.radians(lon - lon0) * EARTH_RADIUS_M * math.cos(math.radians(lat0))
    north = np.radians(lat - lat0) * EARTH_RADIUS_M
    return east, north


def from_enu(east: float, north: float, origin: tuple[float, float]) -> tuple[float, float]:
    lat0, lon0 = origin
    lat = lat0 + math.degrees(north / EARTH_RADIUS_M)
    lon = lon0 + math.degrees(east / (EARTH_RADIUS_M * math.cos(math.radians(lat0))))
    return lat, lon


class GpsData(QObject):
    updated = Signal()
    staleChanged = Signal()
    connectionChanged = Signal()
    lapsChanged = Signal()
    gatesChanged = Signal()

    HISTORY_SAMPLES = 6000  # A minute of the fastest (100 Hz) messages
    TRAIL_SAMPLES = 24000  # Twenty minutes of 20 Hz positions

    def __init__(self, db: Database, parent: QObject | None = None) -> None:
        super().__init__(parent)
        self.db = db
        self.channels = db.signals
        self.messages = {m.name: m for m in db.messages}
        self.values: dict[str, float] = {}
        self.history = {cid: History(self.HISTORY_SAMPLES) for cid in self.channels}
        self.stamps: dict[str, float] = {}  # Message name -> time of its last frame
        self.counts: dict[str, int] = {}
        self.rates: dict[str, float] = {}
        self.stale: set[str] = set(self.messages)  # Message names
        self.unknown_frames = 0
        self.connected = False
        self.link_name = ""

        self.origin: tuple[float, float] | None = None
        self.trail = History(self.TRAIL_SAMPLES)  # t holds latitude, v holds longitude
        self.gates: dict[int, Gate] = {}
        self.laps: list[Lap] = []
        self.events: list[Event] = []
        self.best_lap_ms: int | None = None
        self._last_event: tuple | None = None
        self.missed_laps = False  # Laps completed before the app was listening

        self._t0 = time.monotonic()
        self._rate_counts: dict[str, int] = {}
        self._rate_time = self.now()
        self._timer = QTimer(self)
        self._timer.setInterval(250)
        self._timer.timeout.connect(self._tick)
        self._timer.start()

    # Time is seconds since the app started, so history and plots share one clock.
    def now(self) -> float:
        return time.monotonic() - self._t0

    def ingest(self, batch: list[tuple[str, dict[str, float], float]]) -> None:
        """Take in decoded messages: (message name, signal values, time.monotonic() stamp)."""
        if not batch:
            return
        for name, values, stamp in batch:
            t = stamp - self._t0
            self.stamps[name] = t
            self.counts[name] = self.counts.get(name, 0) + 1
            for signal, value in values.items():
                cid = f"{name}.{signal}"
                self.values[cid] = value
                history = self.history.get(cid)
                if history is not None:
                    history.append(t, value)
            handler = self._handlers.get(name)
            if handler is not None:
                handler(self, values)
        self.check_stale()
        self.updated.emit()

    def value(self, cid: str) -> float | None:
        return self.values.get(cid)

    def flags(self, cid: str) -> int:
        return int(self.values.get(cid, 0))

    def is_stale(self, cid: str) -> bool:
        return cid.split(".", 1)[0] in self.stale

    def age(self, message: str) -> float | None:
        """Seconds since the message last arrived, or None if it never has."""
        stamp = self.stamps.get(message)
        return None if stamp is None else self.now() - stamp

    def stale_after(self, message: str) -> float | None:
        """How long a message may go quiet. None for one that is only sent when something happens."""
        rate = self.messages[message].rate_hz
        return None if rate is None else max(1.0, 3.0 / rate)

    def check_stale(self) -> None:
        now = self.now()
        stale = set()
        for name in self.messages:
            limit = self.stale_after(name)
            if not self.connected:
                stale.add(name)
            elif limit is not None and now - self.stamps.get(name, -1e9) > limit:
                stale.add(name)
        if stale != self.stale:
            self.stale = stale
            self.staleChanged.emit()

    def _tick(self) -> None:
        now = self.now()
        elapsed = now - self._rate_time
        if elapsed >= 1.0:
            self.rates = {name: (count - self._rate_counts.get(name, 0)) / elapsed
                          for name, count in self.counts.items()}
            self._rate_counts = dict(self.counts)
            self._rate_time = now
        self.check_stale()

    def set_connected(self, connected: bool, name: str = "") -> None:
        self.connected = connected
        self.link_name = name if connected else ""
        self.check_stale()
        self.connectionChanged.emit()

    def reset(self) -> None:
        """Forget everything from the previous connection."""
        self.values.clear()
        self.stamps.clear()
        self.counts.clear()
        self.rates.clear()
        self._rate_counts.clear()
        for history in self.history.values():
            history.clear()
        self.unknown_frames = 0
        self.origin = None
        self.trail.clear()
        self.gates.clear()
        self.clear_laps()
        self.set_connected(False)
        self.gatesChanged.emit()
        self.updated.emit()

    # Track

    def position(self) -> tuple[float, float] | None:
        """Metres east and north of the frame origin."""
        lat, lon = self.values.get("GPS_Position.lat_deg"), self.values.get("GPS_Position.lon_deg")
        origin = self.frame_origin()
        if lat is None or lon is None or origin is None:
            return None
        east, north = to_enu(lat, lon, origin)
        return float(east), float(north)

    def frame_origin(self) -> tuple[float, float] | None:
        """The node's own origin once it has sent one; until then, where the trail started."""
        if self.origin is not None:
            return self.origin
        if self.trail.count:
            lat, lon = self.trail.arrays()
            return float(lat[0]), float(lon[0])
        return None

    def trail_enu(self) -> tuple[np.ndarray, np.ndarray]:
        origin = self.frame_origin()
        if origin is None:
            return np.zeros(0), np.zeros(0)
        lat, lon = self.trail.arrays()
        return to_enu(lat, lon, origin)

    def clear_trail(self) -> None:
        self.trail.clear()
        self.updated.emit()

    def _on_position(self, values: dict[str, float]) -> None:
        lat, lon = values["lat_deg"], values["lon_deg"]
        if lat == 0.0 and lon == 0.0:
            return  # The node has no position yet.
        self.trail.append(lat, lon)

    def _on_origin(self, values: dict[str, float]) -> None:
        origin = (values["origin_lat_deg"], values["origin_lon_deg"])
        if origin != (0.0, 0.0) and origin != self.origin:
            self.origin = origin
            self.gatesChanged.emit()

    def _on_gate(self, values: dict[str, float]) -> None:
        index = int(values["gate_index"])
        if not int(values["gate_flags"]) & enums.GATE_VALID:
            if self.gates.pop(index, None) is not None:
                self.gatesChanged.emit()
            return
        gate = Gate(index, values["gate_east_m"], values["gate_north_m"], values["gate_heading_deg"])
        if self.gates.get(index) != gate:
            self.gates[index] = gate
            self.gatesChanged.emit()

    # Laps

    def clear_laps(self) -> None:
        self.laps.clear()
        self.events.clear()
        self.best_lap_ms = None
        self._last_event = None
        self.missed_laps = False
        self.lapsChanged.emit()

    def running(self) -> bool:
        return bool(self.flags("Lap_Status.flags") & enums.LAP_RUNNING) and "Lap_Status.flags" in self.values

    def race_ms(self) -> int | None:
        """Time since the race started: every completed lap plus the one being driven.

        Built from the node's own lap times, so it is as exact as they are. None before the
        first crossing of the start/finish line.
        """
        done = [lap.time_ms for lap in self.laps if lap.time_ms is not None]
        current = int(self.values.get("Lap_Status.running_time_ms", 0)) if self.running() else 0
        if not done and not self.running():
            return None
        return sum(done) + current

    def current_lap(self) -> Lap | None:
        return next((lap for lap in reversed(self.laps) if lap.time_ms is None), None)

    def last_lap(self) -> Lap | None:
        return next((lap for lap in reversed(self.laps) if lap.time_ms is not None), None)

    def _lap(self, number: int) -> Lap:
        for lap in reversed(self.laps):
            if lap.number == number:
                return lap
        lap = Lap(number)
        self.laps.append(lap)
        return lap

    def _on_lap_status(self, values: dict[str, float]) -> None:
        """Keep a row for the lap being driven, from the moment it starts."""
        running = bool(int(values["flags"]) & enums.LAP_RUNNING)
        completed = int(values["lap"])
        done = self.last_lap()
        changed = False
        if done is not None and completed + 1 < done.number:
            # The node's lap count went back by more than a status frame can lag behind an
            # event: the start/finish gate was moved or the node restarted.
            self.laps.clear()
            self.best_lap_ms = None
            done = None
            changed = True
        if running:
            number = completed + 1
            if (done is None or number > done.number) and not any(lap.number == number for lap in self.laps):
                if not self.laps and completed > 0:
                    self.missed_laps = True
                self.laps.append(Lap(number))
                changed = True
        else:
            before = len(self.laps)
            self.laps = [lap for lap in self.laps if lap.time_ms is not None]
            changed |= len(self.laps) != before
        if changed:
            self.lapsChanged.emit()

    def _on_lap_event(self, values: dict[str, float]) -> None:
        kind, number, time_ms = int(values["type"]), int(values["lap"]), int(values["time_ms"])
        key = (int(values["counter"]), kind, number, time_ms)
        if key == self._last_event:
            return  # The same event, received twice.
        self._last_event = key
        self.events.append(Event(time.time(), kind, number, time_ms))
        del self.events[:-500]

        if kind in (enums.EVENT_LAP, enums.EVENT_SECTOR):
            # A sector event carries the laps completed before it, so it belongs to the next lap.
            number = number if kind == enums.EVENT_LAP else number + 1
            done = self.last_lap()
            if done is not None and number <= done.number:
                # The node's lap count went back: the gates were moved or the node restarted.
                self.laps.clear()
                self.best_lap_ms = None
            lap = self._lap(number)
            if kind == enums.EVENT_SECTOR:
                lap.sectors.append(time_ms)
            else:
                lap.time_ms = time_ms
                if self.best_lap_ms is None or time_ms < self.best_lap_ms:
                    self.best_lap_ms = time_ms
        self.lapsChanged.emit()

    _handlers = {
        "GPS_Position": _on_position,
        "GPS_Frame_Origin": _on_origin,
        "GPS_Gate": _on_gate,
        "Lap_Event": _on_lap_event,
        "Lap_Status": _on_lap_status,
    }
