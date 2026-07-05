"""2D lat/lon path plot: driven trail, current position and heading.

Plotted in a local flat-earth metres frame centred on the first fix
seen (not raw lat/lon degrees), so the aspect ratio can be locked 1:1
and a circle drawn on screen actually looks like a circle - GPS.dbc's
own geodesy.c makes the same flat-earth assumption for a track this
size, so this matches what the firmware itself would show.
"""

from __future__ import annotations

import math

import pyqtgraph as pg
from PyQt6.QtWidgets import QVBoxLayout, QWidget

_M_PER_DEG_LAT = 111_320.0
#: Longest trail kept on screen, to bound memory on an unattended
#: multi-hour session rather than growing without limit.
_MAX_TRAIL_POINTS = 20_000
#: Half-length of the heading arrow, in metres.
_ARROW_LEN_M = 3.0


class TrackMap(QWidget):
    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        layout = QVBoxLayout(self)

        self._plot_widget = pg.PlotWidget(title="Track")
        self._plot_widget.setAspectLocked(True)
        self._plot_widget.setLabel("bottom", "east", units="m")
        self._plot_widget.setLabel("left", "north", units="m")
        self._plot_widget.showGrid(x=True, y=True, alpha=0.3)
        layout.addWidget(self._plot_widget)

        self._trail = self._plot_widget.plot(pen=pg.mkPen("c", width=1.5))
        self._position_marker = pg.ScatterPlotItem(
            size=12, brush=pg.mkBrush("r"), pen=pg.mkPen("w")
        )
        self._plot_widget.addItem(self._position_marker)
        self._heading_line = self._plot_widget.plot(pen=pg.mkPen("y", width=2))

        self._origin_lat: float | None = None
        self._origin_lon: float | None = None
        self._m_per_deg_lon = 0.0
        self._east: list[float] = []
        self._north: list[float] = []
        self._heading_deg = 0.0

    def on_message(self, name: str, signals: dict, _timestamp: float) -> None:
        if name == "GPS_Attitude":
            self._heading_deg = signals["yaw_deg"]
            return
        if name != "GPS_Position":
            return

        lat_deg = signals["lat_deg"]
        lon_deg = signals["lon_deg"]
        if self._origin_lat is None:
            self._origin_lat = lat_deg
            self._origin_lon = lon_deg
            self._m_per_deg_lon = _M_PER_DEG_LAT * math.cos(math.radians(lat_deg))

        east = (lon_deg - self._origin_lon) * self._m_per_deg_lon
        north = (lat_deg - self._origin_lat) * _M_PER_DEG_LAT

        self._east.append(east)
        self._north.append(north)
        if len(self._east) > _MAX_TRAIL_POINTS:
            del self._east[0]
            del self._north[0]

        self._trail.setData(self._east, self._north)
        self._position_marker.setData([east], [north])

        heading_rad = math.radians(self._heading_deg)
        tip_east = east + _ARROW_LEN_M * math.sin(heading_rad)
        tip_north = north + _ARROW_LEN_M * math.cos(heading_rad)
        self._heading_line.setData([east, tip_east], [north, tip_north])
