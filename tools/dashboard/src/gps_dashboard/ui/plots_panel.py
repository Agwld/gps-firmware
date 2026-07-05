"""Rolling time-series plots: speed, attitude, accel, gyro.

Each strip keeps a fixed-length rolling buffer keyed by wall-clock
arrival time (not the frame's own timestamp field, which means
different things across live/replay/simulate sources) so the x-axis is
always "seconds ago", comparable across all three source types.
"""

from __future__ import annotations

import time
from collections import deque

import pyqtgraph as pg
from PyQt6.QtWidgets import QVBoxLayout, QWidget

#: How much history each strip keeps on screen.
_WINDOW_S = 30.0
#: Generous upper bound on samples in that window (fastest signal here
#: is 100 Hz IMU data): 100 Hz * 30 s * safety margin.
_MAXLEN = 4000


class _Series:
    def __init__(self, plot: pg.PlotItem, name: str, colour: str) -> None:
        self._t: deque[float] = deque(maxlen=_MAXLEN)
        self._y: deque[float] = deque(maxlen=_MAXLEN)
        self._curve = plot.plot(name=name, pen=pg.mkPen(colour, width=1.5))

    def append(self, value: float) -> None:
        self._t.append(time.monotonic())
        self._y.append(value)

    def redraw(self, now: float) -> None:
        if not self._t:
            return
        xs = [t - now for t in self._t]
        self._curve.setData(xs, list(self._y))


class PlotsPanel(QWidget):
    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        layout = QVBoxLayout(self)

        self._speed_plot = self._make_plot(layout, "Speed", "m/s")
        self._attitude_plot = self._make_plot(layout, "Attitude", "deg")
        self._accel_plot = self._make_plot(layout, "Acceleration", "g")
        self._gyro_plot = self._make_plot(layout, "Angular rate", "deg/s")

        self._series: dict[str, _Series] = {
            "speed": _Series(self._speed_plot, "speed", "c"),
            "yaw": _Series(self._attitude_plot, "yaw", "y"),
            "pitch": _Series(self._attitude_plot, "pitch", "m"),
            "roll": _Series(self._attitude_plot, "roll", "g"),
            "ax": _Series(self._accel_plot, "ax", "r"),
            "ay": _Series(self._accel_plot, "ay", "g"),
            "az": _Series(self._accel_plot, "az", "b"),
            "gx": _Series(self._gyro_plot, "gx", "r"),
            "gy": _Series(self._gyro_plot, "gy", "g"),
            "gz": _Series(self._gyro_plot, "gz", "b"),
        }

        self._redraw_timer = pg.QtCore.QTimer(self)
        self._redraw_timer.timeout.connect(self._redraw_all)
        self._redraw_timer.start(50)

    def _make_plot(self, layout: QVBoxLayout, title: str, y_label: str) -> pg.PlotItem:
        widget = pg.PlotWidget(title=title)
        widget.addLegend()
        widget.setLabel("bottom", "seconds ago")
        widget.setLabel("left", y_label)
        widget.setXRange(-_WINDOW_S, 0.0)
        widget.showGrid(x=True, y=True, alpha=0.3)
        layout.addWidget(widget)
        return widget.getPlotItem()

    def on_message(self, name: str, signals: dict, _timestamp: float) -> None:
        if name == "GPS_Velocity":
            self._series["speed"].append(signals["speed_mps"])
        elif name == "GPS_Attitude":
            self._series["yaw"].append(signals["yaw_deg"])
            self._series["pitch"].append(signals["pitch_deg"])
            self._series["roll"].append(signals["roll_deg"])
        elif name == "GPS_IMU_Accel":
            self._series["ax"].append(signals["ax_mg"] / 1000.0)
            self._series["ay"].append(signals["ay_mg"] / 1000.0)
            self._series["az"].append(signals["az_mg"] / 1000.0)
        elif name == "GPS_IMU_Gyro":
            self._series["gx"].append(signals["gx_dps"])
            self._series["gy"].append(signals["gy_dps"])
            self._series["gz"].append(signals["gz_dps"])

    def _redraw_all(self) -> None:
        now = time.monotonic()
        for series in self._series.values():
            series.redraw(now)
