"""The start screen: connect to the GPS node over CAN, try the simulator, or replay a recording."""

from __future__ import annotations

import threading
from pathlib import Path

from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QPixmap
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QFileDialog,
    QFrame,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QListWidget,
    QListWidgetItem,
    QVBoxLayout,
    QWidget,
)

from gps_desktop import __version__, theme
from gps_desktop.gps import sources
from gps_desktop.ui.widgets import button, label, section_label

# List entries carry "interface\nchannel". These two stand for the entries that are not adapters.
SIMULATOR = "simulator\n"
OTHER = "other\n"
BITRATES = [1_000_000, 500_000, 250_000, 125_000]
SPEEDS = [1.0, 2.0, 5.0, 10.0]


class ConnectionPage(QWidget):
    connectSimulator = Signal()
    connectLive = Signal(str, str, int)  # python-can interface, channel, bit rate
    connectReplay = Signal(str, float, bool)  # Log file, speed, loop
    _detected = Signal(list)

    def __init__(self) -> None:
        super().__init__()
        self.adapters: list[tuple[str, str]] = []
        self.detecting = False
        outer = QVBoxLayout(self)
        outer.addStretch(1)
        row = QHBoxLayout()
        row.addStretch(1)
        card = QFrame()
        card.setObjectName("card")
        card.setFixedWidth(460)
        row.addWidget(card)
        row.addStretch(1)
        outer.addLayout(row)
        outer.addStretch(2)

        box = QVBoxLayout(card)
        box.setContentsMargins(28, 26, 28, 26)
        box.setSpacing(10)

        brand = QHBoxLayout()
        icon = QLabel()
        pixmap = QPixmap(str(Path(__file__).resolve().parent.parent / "assets" / "icon.png"))
        if not pixmap.isNull():
            icon.setPixmap(pixmap.scaled(44, 44, Qt.AspectRatioMode.KeepAspectRatio,
                                         Qt.TransformationMode.SmoothTransformation))
        brand.addWidget(icon)
        titles = QVBoxLayout()
        titles.setSpacing(0)
        title = label("GPS Desktop")
        title.setStyleSheet("font-size: 22px; font-weight: 600;")
        titles.addWidget(title)
        titles.addWidget(label(f"SUFST GPS node · {__version__}", "faint"))
        brand.addLayout(titles)
        brand.addStretch(1)
        box.addLayout(brand)
        box.addSpacing(8)

        box.addWidget(section_label("Connect to the GPS node over CAN"))
        self.list = QListWidget()
        self.list.setMinimumHeight(170)
        self.list.itemDoubleClicked.connect(lambda _item: self._connect())
        self.list.itemSelectionChanged.connect(self._selection_changed)
        box.addWidget(self.list)

        self.manual = QWidget()
        manual = QHBoxLayout(self.manual)
        manual.setContentsMargins(0, 0, 0, 0)
        self.interface = QComboBox()
        self.interface.setEditable(True)
        self.interface.addItems(sources.INTERFACES)
        self.interface.setToolTip("The python-can interface name for your adapter")
        self.channel = QLineEdit()
        self.channel.setPlaceholderText("Channel, such as PCAN_USBBUS1, COM5 or 0")
        manual.addWidget(self.interface)
        manual.addWidget(self.channel, 1)
        box.addWidget(self.manual)

        actions = QHBoxLayout()
        actions.addWidget(button("Refresh", self.refresh))
        self.bitrate = QComboBox()
        for rate in BITRATES:
            self.bitrate.addItem(f"{rate // 1000} kbit/s", rate)
        self.bitrate.setToolTip("The car's sensor bus runs at 1000 kbit/s")
        actions.addWidget(self.bitrate)
        actions.addStretch(1)
        self.connect_button = button("Connect", self._connect, "primary")
        self.connect_button.setMinimumWidth(120)
        actions.addWidget(self.connect_button)
        box.addLayout(actions)

        box.addSpacing(10)
        box.addWidget(section_label("Or replay a recording"))
        replay = QHBoxLayout()
        replay.addWidget(button("Open a log file…", self._open_log), 1)
        self.speed = QComboBox()
        for speed in SPEEDS:
            self.speed.addItem(f"{speed:g}× speed", speed)
        replay.addWidget(self.speed)
        self.loop = QCheckBox("Loop")
        replay.addWidget(self.loop)
        box.addLayout(replay)
        self.error = label("", wrap=True)
        self.error.hide()
        box.addWidget(self.error)

        self._detected.connect(self._on_detected)
        theme.signals.changed.connect(self._fill)
        self._fill()

    def show_error(self, text: str) -> None:
        self.error.setText(text)
        self.error.setStyleSheet(f"color: {theme.FAULT};")
        self.error.setVisible(bool(text))

    def refresh(self) -> None:
        """Look for CAN adapters. Vendor drivers can be slow to answer, so this runs on a thread."""
        if self.detecting:
            return
        self.detecting = True
        self._fill()

        def work() -> None:
            try:
                found = sources.detect_channels()
            except Exception:  # No adapters is an ordinary result, not an error to show.
                found = []
            self._detected.emit(found)

        threading.Thread(target=work, daemon=True).start()

    def _on_detected(self, found: list) -> None:
        self.detecting = False
        self.adapters = found
        self._fill()

    def _fill(self) -> None:
        selected = self.list.currentItem().data(Qt.ItemDataRole.UserRole) if self.list.currentItem() else SIMULATOR
        self.list.clear()

        def add(text: str, data: str | None) -> None:
            item = QListWidgetItem(text)
            if data is None:
                item.setFlags(Qt.ItemFlag.NoItemFlags)
                item.setForeground(theme.color(theme.TEXT_FAINT))
            else:
                item.setData(Qt.ItemDataRole.UserRole, data)
            self.list.addItem(item)
            if data is not None and data == selected:
                self.list.setCurrentItem(item)

        add("Simulated GPS node\nA car lapping a made-up circuit, no hardware needed", SIMULATOR)
        for interface, channel in self.adapters:
            add(f"{channel}\nCAN adapter ({interface})", f"{interface}\n{channel}")
        if self.detecting:
            add("Looking for CAN adapters…", None)
        elif not self.adapters:
            add("No CAN adapters found", None)
        add("Another CAN interface…\nEnter the interface and channel yourself", OTHER)
        if self.list.currentItem() is None:
            self.list.setCurrentRow(0)
        self._selection_changed()

    def _selection_changed(self) -> None:
        item = self.list.currentItem()
        data = item.data(Qt.ItemDataRole.UserRole) if item else None
        self.manual.setVisible(data == OTHER)
        self.bitrate.setEnabled(data is not None and data != SIMULATOR)
        self.connect_button.setEnabled(data is not None)

    def _connect(self) -> None:
        item = self.list.currentItem()
        data = item.data(Qt.ItemDataRole.UserRole) if item else None
        if data is None:
            return
        self.show_error("")
        if data == SIMULATOR:
            self.connectSimulator.emit()
            return
        interface, channel = data.split("\n", 1)
        if data == OTHER:
            interface, channel = self.interface.currentText().strip(), self.channel.text().strip()
            if not interface or not channel:
                self.show_error("Enter both the interface and the channel.")
                return
        self.connectLive.emit(interface, channel, self.bitrate.currentData())

    def _open_log(self) -> None:
        path, _ = QFileDialog.getOpenFileName(self, "Replay a recording", "", "CAN logs (*.log *.txt);;All files (*)")
        if path:
            self.show_error("")
            self.connectReplay.emit(path, self.speed.currentData(), self.loop.isChecked())

    def showEvent(self, event) -> None:
        super().showEvent(event)
        self.refresh()
