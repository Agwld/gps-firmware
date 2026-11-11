"""Runs a source on a background thread and feeds what it decodes into the data store."""

from __future__ import annotations

import threading
from collections import deque

from PySide6.QtCore import QObject, QTimer, Signal

from gps_desktop.gps.dbc import COMMAND_MESSAGE, Database, Frame
from gps_desktop.gps.sources import Recorder, Source
from gps_desktop.gps.store import GpsData

DRAIN_MS = 33  # The store, and so every screen, updates about 30 times a second.


class Link(QObject):
    failed = Signal(str)  # The source stopped with an error, such as an adapter that would not open
    ended = Signal()  # The source ran out by itself, such as a replay reaching the end of its log

    def __init__(self, source: Source, db: Database, data: GpsData, parent: QObject | None = None) -> None:
        super().__init__(parent)
        self.source = source
        self.db = db
        self.data = data
        self.recorder: Recorder | None = None
        self._stop = threading.Event()
        self._lock = threading.Lock()  # Guards the recorder, which the thread writes to
        self._decoded: deque = deque()
        self._outcome: list[str | None] = []  # One entry once the thread has finished
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._timer = QTimer(self)
        self._timer.setInterval(DRAIN_MS)
        self._timer.timeout.connect(self._drain)

    def start(self) -> None:
        self.data.set_connected(True, self.source.name)
        self._thread.start()
        self._timer.start()

    def stop(self) -> None:
        self._stop.set()
        self._timer.stop()
        if self._thread.is_alive():
            self._thread.join(2.0)
        self.record(None)

    def record(self, recorder: Recorder | None) -> None:
        """Start writing frames to this recorder, or stop with None."""
        with self._lock:
            previous, self.recorder = self.recorder, recorder
        if previous is not None:
            previous.close()

    def send(self, frame: Frame) -> bool:
        return self.source.send(frame)

    def _run(self) -> None:
        error = None
        try:
            for frame in self.source.frames(self._stop):
                with self._lock:
                    if self.recorder is not None:
                        self.recorder.write(frame)
                decoded = self.db.decode(frame)
                self._decoded.append((decoded, frame.t))
        except Exception as exception:  # Drivers and files fail in their own ways; report them all.
            error = str(exception) or type(exception).__name__
        self._outcome.append(error)

    def _drain(self) -> None:
        batch = []
        while self._decoded:
            decoded, t = self._decoded.popleft()
            if decoded is None:
                self.data.unknown_frames += 1
            elif decoded[0] != COMMAND_MESSAGE:
                batch.append((decoded[0], decoded[1], t))
        self.data.ingest(batch)
        if self._outcome and not self._decoded:
            self._timer.stop()
            error = self._outcome[0]
            if self._stop.is_set():
                return
            if error is None:
                self.ended.emit()
            else:
                self.failed.emit(error)
