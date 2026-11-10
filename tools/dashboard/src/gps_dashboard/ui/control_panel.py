"""Remote control panel: mirrors the car's steering-wheel lap button (and
a couple of CAN-only commands) so gates can be set from the dashboard.

Emits `command_requested(cmd, arg0, arg1)`; MainWindow encodes that into a
GPS_Command frame and sends it on the active bus. Buttons are disabled
until connected. "Set next sector gate" sends the same slot the lap
button does (CAN_GATE_SLOT_NEXT_SECTOR), so the node picks the first free
sector slot from its own gate table - the dashboard keeps no counter that
could disagree with it (after a reboot, or a press of the real button).
"""

from __future__ import annotations

from PyQt6.QtCore import pyqtSignal
from PyQt6.QtWidgets import (
    QGroupBox,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

# GPS_Command sub-commands and special slots (see can_defs.h).
_CMD_GATE_SET = 0x01
_CMD_GATE_CLEAR = 0x02
_CMD_CONFIG_SAVE = 0x20
_GATE_SLOT_NEXT_SECTOR = 0xFE
_GATE_CLEAR_ALL = 0xFF


class ControlPanel(QGroupBox):
    #: (cmd, arg0, arg1) for a GPS_Command to transmit.
    command_requested = pyqtSignal(int, int, int)

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__("Gate control", parent)

        layout = QVBoxLayout(self)

        self._sf_button = QPushButton("Set start / finish")
        self._sf_button.setToolTip(
            "Place the start/finish line at the car's current position "
            "(clears sector gates, restarts lap timing). Steering wheel: "
            "long press."
        )
        self._sf_button.clicked.connect(
            lambda: self.command_requested.emit(_CMD_GATE_SET, 0, 0)
        )
        layout.addWidget(self._sf_button)

        self._sector_button = QPushButton("Set next sector gate")
        self._sector_button.setToolTip(
            "Place a sector gate at the current position, in the first free "
            "sector slot. Steering wheel: short press."
        )
        self._sector_button.clicked.connect(
            lambda: self.command_requested.emit(
                _CMD_GATE_SET, _GATE_SLOT_NEXT_SECTOR, 0
            )
        )
        layout.addWidget(self._sector_button)

        self._clear_button = QPushButton("Clear all gates")
        self._clear_button.setToolTip(
            "Remove every gate. Steering wheel: very long press."
        )
        self._clear_button.clicked.connect(
            lambda: self.command_requested.emit(
                _CMD_GATE_CLEAR, _GATE_CLEAR_ALL, 0
            )
        )
        layout.addWidget(self._clear_button)

        self._save_button = QPushButton("Compact flash store")
        self._save_button.setToolTip(
            "Gates are saved as they change; this reclaims flash space "
            "(CAN_CMD_CONFIG_SAVE). The node waits until the car is stationary."
        )
        self._save_button.clicked.connect(
            lambda: self.command_requested.emit(_CMD_CONFIG_SAVE, 0, 0)
        )
        layout.addWidget(self._save_button)

        self.set_enabled(False)

    def set_enabled(self, connected: bool) -> None:
        """Enable the buttons only while a bus is connected."""
        for b in (
            self._sf_button,
            self._sector_button,
            self._clear_button,
            self._save_button,
        ):
            b.setEnabled(connected)
