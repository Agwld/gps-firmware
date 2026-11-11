"""Track: where the car has been, where it is and where the timing gates are. Gates are set here."""

from __future__ import annotations

import math

import pyqtgraph as pg
from PySide6.QtCore import Qt
from PySide6.QtWidgets import QCheckBox, QListWidget, QListWidgetItem, QMessageBox

from gps_desktop import theme
from gps_desktop.gps import enums
from gps_desktop.ui.context import AppContext
from gps_desktop.ui.screens.base import Screen
from gps_desktop.ui.widgets import button, colored, label, screen_layout, style_plot

GATE_HALF_WIDTH_M = 2.0  # LAP_GATE_HALF_WIDTH_M in the firmware's board_config.h
HEADING_LINE_M = 4.0
FOLLOW_RANGE_M = 40.0


class TrackScreen(Screen):
    title = "Track"
    nav_label = "Track"

    def __init__(self, ctx: AppContext) -> None:
        super().__init__(ctx)
        box, self.header = screen_layout(self, "Track")
        self.plot = pg.PlotWidget()
        self.plot.setAspectLocked(True)
        self.plot.setLabel("bottom", "East", units="m")
        self.plot.setLabel("left", "North", units="m")
        self.plot.setMenuEnabled(False)
        box.addWidget(self.plot, 1)
        self.trail = self.plot.plot([], [], antialias=True)
        self.heading = self.plot.plot([], [], antialias=True)
        self.car = pg.ScatterPlotItem(size=14)
        self.plot.addItem(self.car)
        self.gate_items: list = []

        self.panel.section("Gates")
        self.command_buttons = [
            self.panel.add(button("Set start/finish here", lambda: self._set_gate(0), "primary",
                                  "Place the start/finish line where the car is now, across its direction of travel")),
            self.panel.add(button("Add a sector gate here", lambda: self._set_gate(enums.SLOT_NEXT_SECTOR),
                                  tooltip="Place the next free sector gate where the car is now")),
        ]
        self.gate_list = self.panel.add(QListWidget())
        self.gate_list.setFixedHeight(190)
        self.gate_list.itemSelectionChanged.connect(self._gates_selected)
        self.clear_one = self.panel.add(button("Clear the selected gate", self._clear_selected))
        self.command_buttons += [
            self.panel.add(button("Clear all gates", self._clear_all, "danger")),
            self.panel.add(button("Save gates to the node", self._save,
                                  tooltip="Keep the gates through a power cycle. The node saves when it is next stationary.")),
        ]
        self.panel.note("Gates are stored by the node, not by this app. They are as wide as the firmware "
                        f"treats them: {GATE_HALF_WIDTH_M:.0f} m either side of the point.")
        self.panel.section("View")
        self.follow = self.panel.add(QCheckBox("Follow the car"))
        self.follow.toggled.connect(self._follow_changed)
        self.panel.add(button("Fit the whole track", self._fit))
        self.panel.add(button("Clear the trail", ctx.data.clear_trail))
        self.panel.section("Position")
        self.readout = self.panel.add(label("", "muted", wrap=True))
        self.panel.finish()

        ctx.data.gatesChanged.connect(self._draw_gates)
        ctx.linkChanged.connect(self._link_changed)
        self.restyle()
        self._link_changed()

    def restyle(self) -> None:
        self.plot.setBackground(theme.BG)
        style_plot(self.plot.getPlotItem())
        self.trail.setPen(pg.mkPen(theme.ACCENT, width=2))
        self.heading.setPen(pg.mkPen(theme.TEXT, width=2))
        self.car.setBrush(pg.mkBrush(theme.TEXT))
        self.car.setPen(pg.mkPen(theme.BG, width=2))
        self._draw_gates()
        self.refresh()

    # Gate commands

    def _link_changed(self) -> None:
        for widget in self.command_buttons:
            widget.setEnabled(self.ctx.connected)
        self._gates_selected()

    def _set_gate(self, slot: int) -> None:
        self.ctx.command(enums.CMD_GATE_SET, slot,
                         done="Start/finish gate set" if slot == 0 else "Sector gate added")

    def _gates_selected(self) -> None:
        self.clear_one.setEnabled(self.ctx.connected and bool(self.gate_list.selectedItems()))

    def _clear_selected(self) -> None:
        item = self.gate_list.currentItem()
        if item is not None:
            slot = item.data(Qt.ItemDataRole.UserRole)
            self.ctx.command(enums.CMD_GATE_CLEAR, slot, done=f"{enums.gate_name(slot)} gate cleared")

    def _clear_all(self) -> None:
        answer = QMessageBox.question(self, "Clear all gates", "Remove the start/finish line and every sector gate?")
        if answer == QMessageBox.StandardButton.Yes:
            self.ctx.command(enums.CMD_GATE_CLEAR, enums.SLOT_ALL, done="All gates cleared")

    def _save(self) -> None:
        self.ctx.command(enums.CMD_CONFIG_SAVE, done="The node will save its gates when it is next stationary")

    # Drawing

    def _follow_changed(self, follow: bool) -> None:
        if follow:
            self.plot.enableAutoRange(False)
        else:
            self._fit()
        self.refresh()

    def _fit(self) -> None:
        self.follow.setChecked(False)
        self.plot.enableAutoRange(True)

    def _draw_gates(self) -> None:
        for item in self.gate_items:
            self.plot.removeItem(item)
        self.gate_items.clear()
        self.gate_list.clear()
        for index, gate in sorted(self.ctx.data.gates.items()):
            hex_code = theme.OK if index == 0 else theme.WARN
            heading = math.radians(gate.heading)
            # The gate line runs across the direction of travel.
            across_e, across_n = math.cos(heading), -math.sin(heading)
            line = self.plot.plot(
                [gate.east - across_e * GATE_HALF_WIDTH_M, gate.east + across_e * GATE_HALF_WIDTH_M],
                [gate.north - across_n * GATE_HALF_WIDTH_M, gate.north + across_n * GATE_HALF_WIDTH_M],
                pen=pg.mkPen(hex_code, width=4), antialias=True)
            text = pg.TextItem("S/F" if index == 0 else f"S{index}", color=hex_code, anchor=(0.5, 1.3))
            text.setPos(gate.east, gate.north)
            self.plot.addItem(text)
            self.gate_items += [line, text]
            item = QListWidgetItem(f"{enums.gate_name(index)}\n{gate.east:+.1f} m E, {gate.north:+.1f} m N, "
                                   f"heading {gate.heading:.0f}°")
            item.setData(Qt.ItemDataRole.UserRole, index)
            self.gate_list.addItem(item)
        self._gates_selected()

    def refresh(self) -> None:
        data = self.ctx.data
        east, north = data.trail_enu()
        self.trail.setData(east, north)
        position = data.position()
        if position is None:
            self.car.setData([], [])
            self.heading.setData([], [])
            self.readout.setText("No position yet")
        else:
            e, n = position
            self.car.setData([e], [n])
            yaw = data.value("GPS_Attitude.yaw_deg")
            if yaw is None:
                self.heading.setData([], [])
            else:
                yaw = math.radians(yaw)
                self.heading.setData([e, e + math.sin(yaw) * HEADING_LINE_M], [n, n + math.cos(yaw) * HEADING_LINE_M])
            if self.follow.isChecked():
                self.plot.setRange(xRange=(e - FOLLOW_RANGE_M, e + FOLLOW_RANGE_M),
                                   yRange=(n - FOLLOW_RANGE_M, n + FOLLOW_RANGE_M), padding=0)
            speed = data.values.get("GPS_Velocity.speed_mps", 0.0)
            self.readout.setText(f"{e:+.1f} m east, {n:+.1f} m north of the origin\n{speed * 3.6:.1f} km/h")

        origin = data.origin
        gates = len(data.gates)
        parts = [f"{gates} gate{'s' if gates != 1 else ''}"]
        parts.append(f"origin {origin[0]:.5f}°, {origin[1]:.5f}°" if origin else
                     colored("the node has not sent its origin yet", theme.WARN) if self.ctx.connected else "")
        self.header.summary.setText(" · ".join(p for p in parts if p))
