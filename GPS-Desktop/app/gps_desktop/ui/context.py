"""Shared state for every screen: the CAN database, live data and the link to the GPS node."""

from __future__ import annotations

from pathlib import Path

from PySide6.QtCore import QObject, Signal

from gps_desktop.gps.dbc import COMMAND_MESSAGE, Database
from gps_desktop.gps.link import Link
from gps_desktop.gps.sources import LiveSource, Recorder, ReplaySource, Source
from gps_desktop.gps.store import GpsData


class AppContext(QObject):
    plotRequested = Signal(str)  # A signal id someone clicked, to add to the Plots screen
    linkChanged = Signal()
    linkFailed = Signal(str)  # The connection dropped with this error
    recordingChanged = Signal()
    message = Signal(str)  # A short status bar message

    def __init__(self, parent: QObject | None = None) -> None:
        super().__init__(parent)
        self.db = Database()
        self.data = GpsData(self.db, self)
        self.link: Link | None = None

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

    def plot(self, cid: str) -> None:
        if cid in self.data.channels:
            self.plotRequested.emit(cid)
