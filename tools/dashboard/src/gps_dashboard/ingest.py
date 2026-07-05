"""Background thread: pulls raw frames from whichever CanSource is
active, decodes them, and re-emits as Qt signals for the GUI thread.

This is the only place a CanSource, a Decoder and (optionally) a
CandumpRecorder meet - live/replay/simulate all look identical from
here on, which is exactly the point of the CanSource abstraction.
"""

from __future__ import annotations

import threading

from PyQt6.QtCore import QThread, pyqtSignal

from gps_dashboard.can_sources.base import CanSource
from gps_dashboard.decoder import Decoder
from gps_dashboard.recorder import CandumpRecorder


class IngestWorker(QThread):
    #: (message_name, decoded_signals, source_timestamp)
    frame_decoded = pyqtSignal(str, dict, float)
    #: arbitration_id of a frame with no matching DBC message
    unknown_frame = pyqtSignal(int)
    #: fatal error from the source (bus open failure, bad log file, ...)
    error = pyqtSignal(str)

    def __init__(
        self,
        source: CanSource,
        decoder: Decoder,
        recorder: CandumpRecorder | None = None,
    ) -> None:
        super().__init__()
        self._source = source
        self._decoder = decoder
        self._recorder = recorder
        self._stop_event = threading.Event()

    def run(self) -> None:
        try:
            for raw in self._source.frames(self._stop_event):
                if self._recorder is not None:
                    self._recorder.write(raw)
                decoded = self._decoder.decode(raw)
                if decoded is None:
                    self.unknown_frame.emit(raw.arbitration_id)
                    continue
                self.frame_decoded.emit(decoded.name, decoded.signals, decoded.timestamp)
        except Exception as exc:  # noqa: BLE001 - talking to external CAN
            # hardware/drivers/files here; an uncaught exception would just
            # kill this thread silently with the UI frozen mid-session, so
            # every backend's own exception type gets surfaced instead.
            self.error.emit(f"{type(exc).__name__}: {exc}")
        finally:
            if self._recorder is not None:
                self._recorder.close()

    def stop(self) -> None:
        """Request a prompt, graceful stop (see CanSource.frames)."""
        self._stop_event.set()
