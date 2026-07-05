"""Top-level window: wires ConnectionPanel -> IngestWorker -> the three
read-only display panels (status/plots/track)."""

from __future__ import annotations

from PyQt6.QtWidgets import QHBoxLayout, QMainWindow, QSplitter, QTabWidget, QWidget
from PyQt6.QtCore import Qt

from gps_dashboard.can_sources.base import CanSource
from gps_dashboard.decoder import Decoder
from gps_dashboard.ingest import IngestWorker
from gps_dashboard.recorder import CandumpRecorder
from gps_dashboard.ui.connection_panel import ConnectionPanel
from gps_dashboard.ui.plots_panel import PlotsPanel
from gps_dashboard.ui.status_panel import StatusPanel
from gps_dashboard.ui.track_map import TrackMap


class MainWindow(QMainWindow):
    def __init__(self, decoder: Decoder | None = None) -> None:
        super().__init__()
        self.setWindowTitle("STAG 12 GPS Dashboard")
        self.resize(1400, 900)

        self._decoder = decoder or Decoder()
        self._worker: IngestWorker | None = None

        self._connection_panel = ConnectionPanel(self._decoder)
        self._status_panel = StatusPanel()
        self._plots_panel = PlotsPanel()
        self._track_map = TrackMap()

        self._connection_panel.connect_requested.connect(self._start_ingest)
        self._connection_panel.disconnect_requested.connect(self._stop_ingest)

        display_tabs = QTabWidget()
        display_tabs.addTab(self._status_panel, "Status")
        display_tabs.addTab(self._plots_panel, "Plots")
        display_tabs.addTab(self._track_map, "Track")

        splitter = QSplitter(Qt.Orientation.Horizontal)
        splitter.addWidget(self._connection_panel)
        splitter.addWidget(display_tabs)
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)
        splitter.setSizes([320, 1080])

        central = QWidget()
        layout = QHBoxLayout(central)
        layout.addWidget(splitter)
        self.setCentralWidget(central)

    # -- ingest lifecycle --------------------------------------------------

    def _start_ingest(self, source: CanSource, record_path: str | None) -> None:
        if self._worker is not None:
            return
        recorder = CandumpRecorder(record_path) if record_path else None
        worker = IngestWorker(source, self._decoder, recorder)
        worker.frame_decoded.connect(self._status_panel.on_message)
        worker.frame_decoded.connect(self._plots_panel.on_message)
        worker.frame_decoded.connect(self._track_map.on_message)
        worker.unknown_frame.connect(self._status_panel.on_unknown_frame)
        worker.error.connect(self._on_worker_error)
        worker.finished.connect(self._on_worker_finished)
        self._worker = worker
        worker.start()
        self._connection_panel.set_connected(True)

    def _stop_ingest(self) -> None:
        if self._worker is None:
            return
        self._worker.stop()
        self._worker.wait(2000)
        self._worker = None
        self._connection_panel.set_connected(False)

    def _on_worker_error(self, message: str) -> None:
        self._connection_panel.set_connected(False, message=f"Error: {message}")
        self._worker = None

    def _on_worker_finished(self) -> None:
        # Only relevant when the source ended on its own (e.g. a
        # non-looping replay reaching EOF) rather than via Disconnect,
        # which already clears _worker and updates the button itself.
        if self._worker is not None and not self._worker.isRunning():
            self._connection_panel.set_connected(False, message="Source ended")
            self._worker = None

    def closeEvent(self, event) -> None:  # noqa: N802 - Qt override
        self._stop_ingest()
        super().closeEvent(event)
