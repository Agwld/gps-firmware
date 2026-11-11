"""Saved locations: a named place, the track lines drawn over its map, and its gate layout.

Everything is stored as latitude and longitude, never as positions relative to the node's
origin, because the node picks a new origin every time it powers up. The file is plain JSON so it
can be copied between laptops with Export and Import.
"""

from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass, field
from pathlib import Path

from PySide6.QtCore import QObject, Signal

VERSION = 1


@dataclass
class PlannedGate:
    slot: int  # 0 = start/finish, 1 to 7 = sectors
    lat: float
    lon: float
    heading: float  # Direction of travel, degrees anticlockwise from east, as the node uses


@dataclass
class Location:
    name: str
    lat: float  # Where the map centres when this location is opened
    lon: float
    lines: list[list[list[float]]] = field(default_factory=list)  # Each line is [[lat, lon], ...]
    gates: list[PlannedGate] = field(default_factory=list)

    def gate(self, slot: int) -> PlannedGate | None:
        return next((g for g in self.gates if g.slot == slot), None)

    def set_gate(self, gate: PlannedGate) -> None:
        """Add or replace a gate. A new start/finish clears the sectors, as it does on the node."""
        if gate.slot == 0:
            self.gates.clear()
        self.gates = [g for g in self.gates if g.slot != gate.slot] + [gate]
        self.gates.sort(key=lambda g: g.slot)

    def clear_gate(self, slot: int) -> None:
        self.gates = [g for g in self.gates if g.slot != slot]

    def next_sector(self) -> int:
        """The slot the node would give a new sector gate."""
        return next((s for s in range(1, 8) if self.gate(s) is None), 7)

    @staticmethod
    def from_dict(d: dict) -> "Location":
        return Location(
            name=str(d["name"]), lat=float(d["lat"]), lon=float(d["lon"]),
            lines=[[[float(p[0]), float(p[1])] for p in line] for line in d.get("lines", []) if len(line) >= 2],
            gates=sorted((PlannedGate(int(g["slot"]), float(g["lat"]), float(g["lon"]), float(g["heading"]))
                          for g in d.get("gates", []) if 0 <= int(g["slot"]) < 8), key=lambda g: g.slot),
        )


class Locations(QObject):
    changed = Signal()

    def __init__(self, path: Path, parent: QObject | None = None) -> None:
        super().__init__(parent)
        self.path = path
        self.items: list[Location] = []
        self.error = ""
        self.load()

    def load(self) -> None:
        try:
            self.items = self.read(self.path)
        except FileNotFoundError:
            self.items = []
        except (OSError, ValueError, KeyError, TypeError) as error:
            # Keep the damaged file rather than overwrite it on the next save.
            self.error = f"Could not read {self.path.name}: {error}"
            self.path.replace(self.path.with_suffix(".damaged.json"))
            self.items = []

    @staticmethod
    def read(path: Path) -> list[Location]:
        document = json.loads(path.read_text(encoding="utf-8"))
        return [Location.from_dict(d) for d in document.get("locations", [])]

    def save(self) -> None:
        self.write(self.path, self.items)
        self.changed.emit()

    @staticmethod
    def write(path: Path, items: list[Location]) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        document = {"version": VERSION, "locations": [asdict(item) for item in items]}
        temporary = path.with_suffix(".tmp")
        temporary.write_text(json.dumps(document, indent=1), encoding="utf-8")
        temporary.replace(path)  # Never leaves a half-written file behind

    def names(self) -> list[str]:
        return [item.name for item in self.items]

    def get(self, name: str) -> Location | None:
        return next((item for item in self.items if item.name == name), None)

    def add(self, location: Location) -> None:
        self.items = [item for item in self.items if item.name != location.name] + [location]
        self.items.sort(key=lambda item: item.name.lower())
        self.save()

    def remove(self, name: str) -> None:
        self.items = [item for item in self.items if item.name != name]
        self.save()

    def rename(self, old: str, new: str) -> None:
        location = self.get(old)
        if location is not None and new and self.get(new) is None:
            location.name = new
            self.items.sort(key=lambda item: item.name.lower())
            self.save()

    def import_file(self, path: Path) -> int:
        """Add the locations in another file. Ones with the same name are replaced."""
        incoming = self.read(path)
        for location in incoming:
            self.items = [item for item in self.items if item.name != location.name] + [location]
        self.items.sort(key=lambda item: item.name.lower())
        self.save()
        return len(incoming)

    def nearest(self, lat: float, lon: float, within_m: float = 3000.0) -> Location | None:
        """The saved location closest to a point, if one is near enough to be the same place."""
        def distance(item: Location) -> float:
            dx = math.radians(item.lon - lon) * math.cos(math.radians(lat))
            return 6371000.0 * math.hypot(dx, math.radians(item.lat - lat))

        best = min(self.items, key=distance, default=None)
        return best if best is not None and distance(best) <= within_m else None
