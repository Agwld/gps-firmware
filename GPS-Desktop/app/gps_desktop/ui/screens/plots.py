"""Plots: any signals against time. Signals with the same unit share a plot."""

from __future__ import annotations

import time

import pyqtgraph as pg
from PySide6.QtCore import Qt
from PySide6.QtWidgets import QTreeWidget, QTreeWidgetItem

from gps_desktop import theme
from gps_desktop.ui.context import AppContext
from gps_desktop.ui.screens.base import Screen
from gps_desktop.ui.widgets import Segmented, button, label, screen_layout, style_plot

WINDOWS = [10, 30, 60]  # Seconds of history to show
REDRAW_INTERVAL = 1 / 15  # Redrawing every curve at the full 30 Hz would cost more than it shows.
PRESETS = {
    "Speed": ["GPS_Velocity.speed_mps", "GPS_Airspeed.airspeed_mps"],
    "Attitude": ["GPS_Attitude.yaw_deg", "GPS_Attitude.pitch_deg", "GPS_Attitude.roll_deg"],
    "Acceleration": ["GPS_IMU_Accel.ax_mg", "GPS_IMU_Accel.ay_mg", "GPS_IMU_Accel.az_mg"],
    "Rotation": ["GPS_IMU_Gyro.gx_dps", "GPS_IMU_Gyro.gy_dps", "GPS_IMU_Gyro.gz_dps"],
    "Magnetometer": ["GPS_Mag.mx_ut", "GPS_Mag.my_ut", "GPS_Mag.mz_ut"],
    "Accuracy": ["GPS_Quality.hacc_mm", "GPS_Quality.pdop"],
    "Temperatures": ["GPS_Temp.mcp9800_temp_c", "GPS_Temp.imu_temp_c", "GPS_Temp.mcu_temp_c"],
}


class PlotsScreen(Screen):
    title = "Plots"
    nav_label = "Plots"

    def __init__(self, ctx: AppContext) -> None:
        super().__init__(ctx)
        box, self.header = screen_layout(self, "Plots")
        self.graphics = pg.GraphicsLayoutWidget()
        box.addWidget(self.graphics, 1)
        self.empty = label("Tick signals on the left, or pick a preset, to plot them.", "muted")
        self.empty.setAlignment(Qt.AlignmentFlag.AlignCenter)
        box.addWidget(self.empty, 1)
        self.curves: dict[str, pg.PlotDataItem] = {}
        self.plots: list[pg.PlotItem] = []
        self.paused = False
        self.frozen_at = 0.0
        self.last_draw = 0.0

        self.panel.section("Presets")
        for name, cids in PRESETS.items():
            self.panel.add(button(name, lambda cids=cids: self.select(cids)))
        self.panel.section("Time shown")
        self.window = self.panel.add(Segmented([f"{s} s" for s in WINDOWS], current=1))
        self.window.changed.connect(lambda _index: self.refresh(force=True))
        self.pause_button = self.panel.add(button("Pause", self._toggle_pause))
        self.pause_button.setCheckable(True)
        self.panel.section("Signals")
        self.tree = self.panel.add(QTreeWidget())
        self.tree.setHeaderHidden(True)
        self.tree.setMinimumHeight(320)
        self.items: dict[str, QTreeWidgetItem] = {}
        for message in ctx.db.messages:
            parent = QTreeWidgetItem(self.tree, [message.name])
            parent.setToolTip(0, message.comment)
            for signal in message.signals:
                item = QTreeWidgetItem(parent, [f"{signal.name}  ({signal.unit})" if signal.unit else signal.name])
                item.setFlags(item.flags() | Qt.ItemFlag.ItemIsUserCheckable)
                item.setCheckState(0, Qt.CheckState.Unchecked)
                item.setToolTip(0, signal.comment or message.comment)
                self.items[signal.cid] = item
        self.tree.itemChanged.connect(lambda _item, _column: self._rebuild())
        self.panel.add(button("Clear all", lambda: self.select([])))
        self.panel.finish()

        ctx.plotRequested.connect(self.add)
        self.select(PRESETS["Speed"])

    def selected(self) -> list[str]:
        return [cid for cid, item in self.items.items() if item.checkState(0) == Qt.CheckState.Checked]

    def select(self, cids: list[str]) -> None:
        """Plot exactly these signals."""
        self.tree.blockSignals(True)
        for cid, item in self.items.items():
            item.setCheckState(0, Qt.CheckState.Checked if cid in cids else Qt.CheckState.Unchecked)
        self.tree.blockSignals(False)
        self._rebuild()

    def add(self, cid: str) -> None:
        if cid in self.items:
            self.items[cid].setCheckState(0, Qt.CheckState.Checked)
            self.ctx.message.emit(f"Added {cid} to Plots")

    def _toggle_pause(self) -> None:
        self.paused = self.pause_button.isChecked()
        self.pause_button.setText("Resume" if self.paused else "Pause")
        self.frozen_at = self.ctx.data.now()
        self.refresh(force=True)

    def _rebuild(self) -> None:
        """Lay the plots out again: one per unit, sharing a time axis."""
        self.graphics.clear()
        self.curves.clear()
        self.plots.clear()
        selected = self.selected()
        self.empty.setVisible(not selected)
        self.graphics.setVisible(bool(selected))
        by_unit: dict[str, list[str]] = {}
        for cid in selected:
            by_unit.setdefault(self.ctx.data.channels[cid].unit, []).append(cid)
        color = 0
        for row, (unit, cids) in enumerate(by_unit.items()):
            plot = self.graphics.addPlot(row=row, col=0)
            plot.setMenuEnabled(False)
            plot.setMouseEnabled(x=False, y=True)
            plot.setLabel("left", unit or "value")
            plot.addLegend(offset=(8, 8))
            if self.plots:
                plot.setXLink(self.plots[0])
            for cid in cids:
                info = self.ctx.data.channels[cid]
                pen = pg.mkPen(theme.SERIES[color % len(theme.SERIES)], width=1.5)
                self.curves[cid] = plot.plot([], [], pen=pen, name=f"{info.message}.{info.name}")
                color += 1
            self.plots.append(plot)
        if self.plots:
            self.plots[-1].setLabel("bottom", "Seconds ago")
        self.restyle()

    def restyle(self) -> None:
        self.graphics.setBackground(theme.BG)
        for plot in self.plots:
            style_plot(plot)
        for i, curve in enumerate(self.curves.values()):
            curve.setPen(pg.mkPen(theme.SERIES[i % len(theme.SERIES)], width=1.5))
        self.refresh(force=True)

    def refresh(self, force: bool = False) -> None:
        if not self.curves:
            self.header.summary.setText("")
            return
        if self.paused and not force:
            return
        wall = time.monotonic()
        if not force and wall - self.last_draw < REDRAW_INTERVAL:
            return
        self.last_draw = wall
        data = self.ctx.data
        now = self.frozen_at if self.paused else data.now()
        span = WINDOWS[self.window.current()]
        for cid, curve in self.curves.items():
            t, v = data.history[cid].arrays()
            keep = (t >= now - span) & (t <= now)
            curve.setData(t[keep] - now, v[keep])
        self.plots[0].setXRange(-span, 0, padding=0)
        count = len(self.curves)
        self.header.summary.setText(f"{count} signal{'s' if count != 1 else ''}" + (" · paused" if self.paused else ""))
