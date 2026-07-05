"""Remote control panel: mirrors the car's steering-wheel lap button (and
a couple of CAN-only commands) so gates can be set from the dashboard.

Emits `command_requested(cmd, arg0, arg1)`; MainWindow encodes that into a
GPS_Command frame and sends it on the active bus. Buttons are disabled
until connected. The "New sector" button reproduces the firmware's
button FSM (an auto-incrementing sector slot, reset when a new
start/finish is set or all gates are cleared).
"""

from __future__ import annotations

from PyQt6.QtCore import pyqtSignal
from PyQt6.QtWidgets import (
    QGroupBox,
    QLabel,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

# GPS_Command sub-commands (see CAN_CMD_* in can_defs.h).
_CMD_GATE_SET = 0x01
_CMD_GATE_CLEAR = 0x02
_CMD_CONFIG_SAVE = 0x20
_GATE_CLEAR_ALL = 0xFF

# Slots: 0 = start/finish, 1..LAP_MAX_GATES-1 = sectors.
_MAX_GATES = 8


class ControlPanel(QGroupBox):
    #: (cmd, arg0, arg1) for a GPS_Command to transmit.
    command_requested = pyqtSignal(int, int, int)

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__("Gate control", parent)
        self._next_sector = 1

        layout = QVBoxLayout(self)

        self._sf_button = QPushButton("Set start / finish")
        self._sf_button.setToolTip(
            "Place the start/finish line at the car's current position "
            "(clears sector gates). Steering wheel: long press."
        )
        self._sf_button.clicked.connect(self._set_start_finish)
        layout.addWidget(self._sf_button)

        self._sector_button = QPushButton("Set next sector gate")
        self._sector_button.setToolTip(
            "Place the next sector gate at the current position. Steering "
            "wheel: short press."
        )
        self._sector_button.clicked.connect(self._set_sector)
        layout.addWidget(self._sector_button)

        self._clear_button = QPushButton("Clear all gates")
        self._clear_button.setToolTip(
            "Remove every gate. Steering wheel: very long press."
        )
        self._clear_button.clicked.connect(self._clear_all)
        layout.addWidget(self._clear_button)

        self._save_button = QPushButton("Save to flash")
        self._save_button.setToolTip(
            "Persist/compact the gate store now (CAN_CMD_CONFIG_SAVE); the "
            "node only compacts when it judges the car stationary."
        )
        self._save_button.clicked.connect(
            lambda: self.command_requested.emit(_CMD_CONFIG_SAVE, 0, 0)
        )
        layout.addWidget(self._save_button)

        self._hint = QLabel("Next sector: 1")
        layout.addWidget(self._hint)

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

    # -- button handlers ---------------------------------------------------

    def _set_start_finish(self) -> None:
        self.command_requested.emit(_CMD_GATE_SET, 0, 0)
        self._next_sector = 1
        self._update_hint()

    def _set_sector(self) -> None:
        self.command_requested.emit(_CMD_GATE_SET, self._next_sector, 0)
        if self._next_sector < _MAX_GATES - 1:
            self._next_sector += 1
        self._update_hint()

    def _clear_all(self) -> None:
        self.command_requested.emit(_CMD_GATE_CLEAR, _GATE_CLEAR_ALL, 0)
        self._next_sector = 1
        self._update_hint()

    def _update_hint(self) -> None:
        self._hint.setText(f"Next sector: {self._next_sector}")
