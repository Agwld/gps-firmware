"""Shared state for every screen: the CAN database, live data, saved locations, map tiles and
the link to the GPS node."""

from __future__ import annotations

import math
from pathlib import Path

from PySide6.QtCore import QObject, QStandardPaths, QTimer, Signal

from gps_desktop import __version__
from gps_desktop.gps import enums
from gps_desktop.gps.dbc import COMMAND_MESSAGE, GATE_SET_MESSAGE, Database
from gps_desktop.gps.link import Link
from gps_desktop.gps.locations import Location, Locations, PlannedGate
from gps_desktop.gps.sources import LiveSource, Recorder, ReplaySource, Source
from gps_desktop.gps.store import GpsData, from_enu, to_enu
from gps_desktop.gps.tiles import TileCache

# How closely the node's broadcast gate must match the layout to count as the same. The bus
# carries 0.1 m and 0.1 degree steps, and the node and this app convert latitude and longitude
# slightly differently, so allow a little more than that.
MATCH_METRES = 0.35
MATCH_DEGREES = 1.0
VERIFY_AFTER_MS = 3000  # The node re-broadcasts its whole gate set every 1.6 s


def data_folder(kind: QStandardPaths.StandardLocation) -> Path:
    """Per-user storage. Never the install folder, which the installer replaces on update."""
    return Path(QStandardPaths.writableLocation(kind))


class AppContext(QObject):
    plotRequested = Signal(str)  # A signal id someone clicked, to add to the Plots screen
    linkChanged = Signal()
    linkFailed = Signal(str)  # The connection dropped with this error
    recordingChanged = Signal()
    message = Signal(str)  # A short status bar message
    uploadFinished = Signal(bool, str)  # A gate layout was sent: confirmed by the node, and what happened

    def __init__(self, parent: QObject | None = None, folder: Path | None = None) -> None:
        super().__init__(parent)
        self.db = Database()
        self.data = GpsData(self.db, self)
        self.link: Link | None = None
        data = folder or data_folder(QStandardPaths.StandardLocation.AppDataLocation)
        cache = folder / "cache" if folder else data_folder(QStandardPaths.StandardLocation.CacheLocation)
        self.locations = Locations(data / "locations.json", self)
        self.tiles = TileCache(cache / "tiles", f"GPSDesktop/{__version__}", self)
        self._upload: Location | None = None

    @property
    def connected(self) -> bool:
        return self.link is not None

    @property
    def recording(self) -> Path | None:
        recorder = self.link.recorder if self.link else None
        return recorder.path if recorder else None

    def connect_source(self, source: Source) -> None:
        self.disconnect_link()
        self.link = Link(source, self.db, self.data, self)
        self.link.failed.connect(self._on_failed)
        self.link.ended.connect(self._on_ended)
        self.link.start()
        self.linkChanged.emit()

    def connect_simulator(self) -> None:
        from gps_desktop.gps.simulator import Simulator

        self.connect_source(Simulator(self.db))

    def connect_live(self, interface: str, channel: str, bitrate: int) -> None:
        self.connect_source(LiveSource(interface, channel, bitrate))

    def connect_replay(self, path: str, speed: float = 1.0, loop: bool = False) -> None:
        self.connect_source(ReplaySource(path, speed, loop))

    def disconnect_link(self, forget: bool = True) -> None:
        """Close the link. `forget` also clears the data it brought in."""
        if self.link is not None:
            was_recording = self.link.recorder is not None
            self.link.stop()
            self.link.blockSignals(True)
            self.link.deleteLater()
            self.link = None
            if was_recording:
                self.recordingChanged.emit()
        if forget:
            self.data.reset()
        else:
            self.data.set_connected(False)
        self.linkChanged.emit()

    def _on_failed(self, error: str) -> None:
        self.disconnect_link()
        self.linkFailed.emit(error)

    def _on_ended(self) -> None:
        # Keep what a finished replay showed, so its laps and track can still be read.
        name = self.data.link_name
        self.disconnect_link(forget=False)
        self.message.emit(f"{name} finished")

    def command(self, command: int, arg0: int = 0, arg1: int = 0, done: str = "") -> bool:
        """Send a command to the GPS node. Returns False when it could not be sent."""
        if self.link is None:
            self.message.emit("Not connected to a GPS node")
            return False
        frame = self.db.encode(COMMAND_MESSAGE, {"cmd": command, "arg0": arg0, "arg1": arg1})
        try:
            sent = self.link.send(frame)
        except Exception as error:  # Adapter drivers raise their own error types.
            self.message.emit(f"Could not send the command: {error}")
            return False
        self.message.emit((done or "Command sent") if sent else "This connection is read-only")
        return sent

    def start_recording(self, path: str) -> bool:
        if self.link is None:
            return False
        try:
            self.link.record(Recorder(path))
        except OSError as error:
            self.message.emit(f"Could not record to {path}: {error}")
            return False
        self.recordingChanged.emit()
        return True

    def stop_recording(self) -> None:
        if self.link is not None and self.link.recorder is not None:
            frames, path = self.link.recorder.frames, self.link.recorder.path
            self.link.record(None)
            self.message.emit(f"Saved {frames} frames to {path.name}")
            self.recordingChanged.emit()

    # Gates by position

    def node_origin(self) -> tuple[float, float] | None:
        """The node's own frame origin. Gates sent to the node are relative to it."""
        return self.data.origin

    def can_place_gates(self) -> bool:
        return self.connected and self.node_origin() is not None

    def node_gate_latlon(self, slot: int) -> tuple[float, float, float] | None:
        """A gate the node reports, as latitude, longitude and heading."""
        gate = self.data.gates.get(slot)
        origin = self.node_origin()
        if gate is None or origin is None:
            return None
        lat, lon = from_enu(gate.east, gate.north, origin)
        return lat, lon, gate.heading

    def set_gate_at(self, slot: int, lat: float, lon: float, heading: float, quiet: bool = False) -> bool:
        """Place a gate on the node at a position. `heading` is degrees anticlockwise from east."""
        origin = self.node_origin()
        if not self.connected or origin is None:
            if not quiet:
                self.message.emit("Gates can only be sent once the node has a GPS position")
            return False
        east, north = to_enu(lat, lon, origin)
        # The bus carries tenths of a degree in [0, 360): 359.96 must go as 0.0, not 360.0.
        heading = round(heading % 360, 1) % 360
        frame = self.db.encode(GATE_SET_MESSAGE, {"set_index": slot, "set_flags": 0, "set_east_m": float(east),
                                                  "set_north_m": float(north), "set_heading_deg": heading})
        try:
            sent = self.link.send(frame)
        except Exception as error:  # Adapter drivers raise their own error types.
            self.message.emit(f"Could not send the gate: {error}")
            return False
        if not quiet:
            self.message.emit(f"{enums.gate_name(slot)} gate sent to the node" if sent else "This connection is read-only")
        return sent

    def layout_differences(self, location: Location) -> list[str]:
        """How the node's gates differ from a saved layout. Empty when they match."""
        differences = []
        for slot in range(enums.GATE_COUNT):
            planned, actual = location.gate(slot), self.node_gate_latlon(slot)
            name = enums.gate_name(slot)
            if planned is None and actual is None:
                continue
            if planned is None:
                differences.append(f"{name} is on the node but not in the layout")
            elif actual is None:
                differences.append(f"{name} is not on the node")
            else:
                lat, lon, heading = actual
                east, north = to_enu(planned.lat, planned.lon, (lat, lon))
                turn = abs((planned.heading - heading + 180) % 360 - 180)
                if math.hypot(east, north) > MATCH_METRES or turn > MATCH_DEGREES:
                    differences.append(f"{name} is in a different place on the node")
        return differences

    def send_layout(self, location: Location) -> bool:
        """Replace the node's gates with a saved layout, then check what the node reports back."""
        if not self.can_place_gates():
            self.message.emit("Connect to the node, and wait for it to have a GPS position, to send gates")
            return False
        if location.gate(0) is None:
            # Start/finish clears the sectors by itself; without one, clear them explicitly.
            self.command(enums.CMD_GATE_CLEAR, enums.SLOT_ALL, done="Clearing the node's gates")
        for gate in sorted(location.gates, key=lambda g: g.slot):
            if not self.set_gate_at(gate.slot, gate.lat, gate.lon, gate.heading, quiet=True):
                return False
        self._upload = location
        self.message.emit(f"Sending {len(location.gates)} gates to the node…")
        QTimer.singleShot(VERIFY_AFTER_MS, self._verify_upload)
        return True

    def _verify_upload(self) -> None:
        location, self._upload = self._upload, None
        if location is None:
            return
        if not self.connected:
            self.uploadFinished.emit(False, "The connection closed before the node confirmed the gates.")
            return
        differences = self.layout_differences(location)
        if differences:
            self.uploadFinished.emit(False, "The node did not confirm the layout: " + "; ".join(differences) + ".")
        else:
            self.uploadFinished.emit(True, f"The node has the {location.name} gates.")

    def copy_node_gates(self, location: Location) -> int:
        """Save the node's current gates as a location's layout."""
        location.gates = []
        for slot in range(enums.GATE_COUNT):
            actual = self.node_gate_latlon(slot)
            if actual is not None:
                location.gates.append(PlannedGate(slot, *actual))
        self.locations.save()
        return len(location.gates)

    def plot(self, cid: str) -> None:
        if cid in self.data.channels:
            self.plotRequested.emit(cid)
