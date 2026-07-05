"""Source selection and connect/disconnect control.

Owns no CAN/decoding logic itself - it just knows how to turn the
widgets' current state into a `CanSource` plus an optional record path,
via `build_source()`. MainWindow is what actually starts/stops the
IngestWorker.
"""

from __future__ import annotations

from PyQt6.QtCore import pyqtSignal
from PyQt6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFileDialog,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QSpinBox,
    QStackedWidget,
    QVBoxLayout,
    QWidget,
)

from gps_dashboard.can_sources.base import CanSource
from gps_dashboard.can_sources.live import DEFAULT_BITRATE, KNOWN_INTERFACES, LiveCanSource
from gps_dashboard.can_sources.replay import ReplayCanSource
from gps_dashboard.can_sources.simulate import SimulateCanSource
from gps_dashboard.decoder import Decoder


class ConnectionPanel(QWidget):
    #: Emitted with (CanSource, record_path_or_None) when Connect is clicked.
    connect_requested = pyqtSignal(object, object)
    disconnect_requested = pyqtSignal()

    def __init__(self, decoder: Decoder, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._decoder = decoder
        self._connected = False

        layout = QVBoxLayout(self)

        self._mode = QComboBox()
        self._mode.addItems(["Live", "Replay", "Simulate"])
        self._mode.currentIndexChanged.connect(self._on_mode_changed)
        layout.addWidget(self._mode)

        self._stack = QStackedWidget()
        self._stack.addWidget(self._build_live_page())
        self._stack.addWidget(self._build_replay_page())
        self._stack.addWidget(self._build_simulate_page())
        layout.addWidget(self._stack)

        record_row = QHBoxLayout()
        self._record_checkbox = QCheckBox("Record to file")
        self._record_path = QLineEdit()
        self._record_path.setPlaceholderText("candump-format .log output path")
        record_browse = QPushButton("Browse...")
        record_browse.clicked.connect(self._browse_record_path)
        record_row.addWidget(self._record_checkbox)
        record_row.addWidget(self._record_path, 1)
        record_row.addWidget(record_browse)
        layout.addLayout(record_row)

        self._connect_button = QPushButton("Connect")
        self._connect_button.clicked.connect(self._on_connect_clicked)
        layout.addWidget(self._connect_button)

        self._status_label = QLabel("Disconnected")
        layout.addWidget(self._status_label)
        layout.addStretch(1)

    # -- pages ---------------------------------------------------------

    def _build_live_page(self) -> QWidget:
        page = QGroupBox("Live")
        form = QFormLayout(page)

        self._live_interface = QComboBox()
        self._live_interface.setEditable(True)
        self._live_interface.addItems(KNOWN_INTERFACES)
        form.addRow("Interface", self._live_interface)

        self._live_channel = QLineEdit()
        self._live_channel.setPlaceholderText("can0 / 0 / PCAN_USBBUS1 ...")
        form.addRow("Channel", self._live_channel)

        self._live_bitrate = QSpinBox()
        self._live_bitrate.setRange(1000, 8_000_000)
        self._live_bitrate.setSingleStep(1000)
        self._live_bitrate.setValue(DEFAULT_BITRATE)
        form.addRow("Bitrate", self._live_bitrate)

        return page

    def _build_replay_page(self) -> QWidget:
        page = QGroupBox("Replay")
        form = QFormLayout(page)

        path_row = QHBoxLayout()
        self._replay_path = QLineEdit()
        self._replay_path.setPlaceholderText("candump -l log file")
        browse = QPushButton("Browse...")
        browse.clicked.connect(self._browse_replay_path)
        path_row.addWidget(self._replay_path, 1)
        path_row.addWidget(browse)
        form.addRow("File", path_row)

        self._replay_speed = QDoubleSpinBox()
        self._replay_speed.setRange(0.0, 100.0)
        self._replay_speed.setValue(1.0)
        self._replay_speed.setSingleStep(0.5)
        self._replay_speed.setToolTip("0 = as fast as possible, no inter-frame delay")
        form.addRow("Speed", self._replay_speed)

        self._replay_loop = QCheckBox("Loop")
        self._replay_loop.setChecked(True)
        form.addRow("", self._replay_loop)

        return page

    def _build_simulate_page(self) -> QWidget:
        page = QGroupBox("Simulate")
        form = QFormLayout(page)

        self._sim_radius = QDoubleSpinBox()
        self._sim_radius.setRange(5.0, 500.0)
        self._sim_radius.setValue(30.0)
        self._sim_radius.setSuffix(" m")
        form.addRow("Track radius", self._sim_radius)

        self._sim_lap_period = QDoubleSpinBox()
        self._sim_lap_period.setRange(3.0, 300.0)
        self._sim_lap_period.setValue(20.0)
        self._sim_lap_period.setSuffix(" s")
        form.addRow("Lap period", self._sim_lap_period)

        return page

    # -- actions ---------------------------------------------------------

    def _on_mode_changed(self, index: int) -> None:
        self._stack.setCurrentIndex(index)

    def _browse_replay_path(self) -> None:
        path, _ = QFileDialog.getOpenFileName(self, "Select candump log", "", "Log files (*.log *.txt);;All files (*)")
        if path:
            self._replay_path.setText(path)

    def _browse_record_path(self) -> None:
        path, _ = QFileDialog.getSaveFileName(self, "Select record output", "", "Log files (*.log);;All files (*)")
        if path:
            self._record_path.setText(path)

    def _on_connect_clicked(self) -> None:
        if self._connected:
            self.disconnect_requested.emit()
            return
        try:
            source = self._build_source()
        except ValueError as exc:
            self._status_label.setText(f"Error: {exc}")
            return
        record_path = self._record_path.text().strip() if self._record_checkbox.isChecked() else None
        if self._record_checkbox.isChecked() and not record_path:
            self._status_label.setText("Error: record enabled but no output path given")
            return
        self.connect_requested.emit(source, record_path)

    def _build_source(self) -> CanSource:
        mode = self._mode.currentText()
        if mode == "Live":
            channel = self._live_channel.text().strip()
            if not channel:
                raise ValueError("live mode needs a channel")
            return LiveCanSource(
                interface=self._live_interface.currentText().strip(),
                channel=channel,
                bitrate=self._live_bitrate.value(),
            )
        if mode == "Replay":
            path = self._replay_path.text().strip()
            if not path:
                raise ValueError("replay mode needs a log file")
            return ReplayCanSource(
                path, speed=self._replay_speed.value(), loop=self._replay_loop.isChecked()
            )
        if mode == "Simulate":
            return SimulateCanSource(
                self._decoder,
                track_radius_m=self._sim_radius.value(),
                lap_period_s=self._sim_lap_period.value(),
            )
        raise ValueError(f"unknown mode {mode!r}")

    # -- state updates from MainWindow -----------------------------------

    def set_connected(self, connected: bool, message: str = "") -> None:
        self._connected = connected
        self._connect_button.setText("Disconnect" if connected else "Connect")
        self._mode.setEnabled(not connected)
        if message:
            self._status_label.setText(message)
        else:
            self._status_label.setText("Connected" if connected else "Disconnected")
