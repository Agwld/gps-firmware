"""2D lat/lon path plot: driven trail, current position/heading, plus the
lap gates and start/finish line broadcast by the node.

Plotted in a local flat-earth metres frame centred on the first fix seen
(not raw lat/lon degrees), so the aspect ratio can be locked 1:1 and a
circle drawn on screen actually looks like a circle - GPS.dbc's own
geodesy.c makes the same flat-earth assumption for a track this size, so
this matches what the firmware itself would show.

Gates arrive (GPS_Gate) as ENU metres relative to the node's own frame
origin (GPS_Frame_Origin), which is re-anchored every power-up. To draw
them in this widget's frame we lift each gate back to absolute lat/lon
via the node origin, then project it through the same _project() the
trail uses - so gates and trail always line up even though the node and
the dashboard picked their origins independently.
"""

from __future__ import annotations

import math

import pyqtgraph as pg
from PyQt6.QtWidgets import QVBoxLayout, QWidget

from gps_dashboard.track_memory import TrackStore, gate_fingerprint

_M_PER_DEG_LAT = 111_320.0
#: Longest trail kept on screen, to bound memory on an unattended
#: multi-hour session rather than growing without limit.
_MAX_TRAIL_POINTS = 20_000
#: Half-length of the heading arrow, in metres.
_ARROW_LEN_M = 3.0
#: Half-width of a drawn gate line, in metres (matches the firmware's
#: LAP_GATE_HALF_WIDTH_M crossing window).
_GATE_HALF_WIDTH_M = 2.0


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

        # Draw order: gate lines under the trail, trail under the marker.
        self._sector_lines = self._plot_widget.plot(pen=pg.mkPen("y", width=2))
        self._startfinish_line = self._plot_widget.plot(
            pen=pg.mkPen("g", width=3)
        )
        self._gate_markers = pg.ScatterPlotItem(
            size=8, brush=pg.mkBrush(255, 255, 255, 160), pen=pg.mkPen("k")
        )
        self._plot_widget.addItem(self._gate_markers)

        # Faint outline reloaded from a previous session for this track
        # (drawn under the live trail).
        self._reference_trail = self._plot_widget.plot(
            pen=pg.mkPen(100, 100, 100, 160, width=1.0)
        )
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

        # Node's ENU frame origin (from GPS_Frame_Origin) and the latest
        # gate per slot (from GPS_Gate).
        self._node_origin_lat: float | None = None
        self._node_origin_lon: float | None = None
        self._gates: dict[int, dict[str, float]] = {}

        # Remembered outlines, keyed by gate layout: reload the matching
        # track's shape on startup so it isn't blank until the first lap.
        self._store = TrackStore()
        self._fingerprint: str | None = None

    def _ensure_origin(self, lat_deg: float, lon_deg: float) -> None:
        if self._origin_lat is None:
            self._origin_lat = lat_deg
            self._origin_lon = lon_deg
            self._m_per_deg_lon = _M_PER_DEG_LAT * math.cos(math.radians(lat_deg))

    def _project(self, lat_deg: float, lon_deg: float) -> tuple[float, float]:
        """Absolute lat/lon -> local east/north metres in this widget's
        frame. Only valid once _ensure_origin() has run."""
        east = (lon_deg - self._origin_lon) * self._m_per_deg_lon
        north = (lat_deg - self._origin_lat) * _M_PER_DEG_LAT
        return east, north

    def on_message(self, name: str, signals: dict, _timestamp: float) -> None:
        if name == "GPS_Attitude":
            self._heading_deg = signals["yaw_deg"]
            return
        if name == "GPS_Frame_Origin":
            self._node_origin_lat = signals["origin_lat_deg"]
            self._node_origin_lon = signals["origin_lon_deg"]
            self._redraw_gates()
            return
        if name == "GPS_Gate":
            idx = int(signals["gate_index"])
            if signals["gate_flags"] & 0x01:
                self._gates[idx] = {
                    "east_m": signals["gate_east_m"],
                    "north_m": signals["gate_north_m"],
                    "heading_deg": signals["gate_heading_deg"],
                }
            else:
                self._gates.pop(idx, None)  # slot cleared -> remove it
            self._redraw_gates()
            return
        if name != "GPS_Position":
            return

        lat_deg = signals["lat_deg"]
        lon_deg = signals["lon_deg"]
        self._ensure_origin(lat_deg, lon_deg)

        east, north = self._project(lat_deg, lon_deg)
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

    def _redraw_gates(self) -> None:
        """Rebuild the gate line/marker items from the current gate set.
        Needs both this widget's origin (from a position fix) and the
        node's origin (from GPS_Frame_Origin) to place gates."""
        if (
            self._origin_lat is None
            or self._node_origin_lat is None
        ):
            return

        node_m_per_deg_lon = _M_PER_DEG_LAT * math.cos(
            math.radians(self._node_origin_lat)
        )

        sf_x: list[float] = []
        sf_y: list[float] = []
        sector_x: list[float] = []
        sector_y: list[float] = []
        marker_x: list[float] = []
        marker_y: list[float] = []

        for idx, g in sorted(self._gates.items()):
            # Node ENU -> absolute lat/lon -> this widget's frame.
            gate_lat = self._node_origin_lat + g["north_m"] / _M_PER_DEG_LAT
            gate_lon = self._node_origin_lon + g["east_m"] / node_m_per_deg_lon
            cx, cy = self._project(gate_lat, gate_lon)

            # Gate line runs perpendicular to the travel direction (an ENU
            # math bearing, 0 = east, CCW - matching gates.c).
            h = math.radians(g["heading_deg"])
            perp_e = -math.sin(h)
            perp_n = math.cos(h)
            x0 = cx - _GATE_HALF_WIDTH_M * perp_e
            y0 = cy - _GATE_HALF_WIDTH_M * perp_n
            x1 = cx + _GATE_HALF_WIDTH_M * perp_e
            y1 = cy + _GATE_HALF_WIDTH_M * perp_n

            marker_x.append(cx)
            marker_y.append(cy)
            if idx == 0:
                # Separate items can't share a segment; start/finish is its
                # own (single) line so it gets its own colour.
                sf_x += [x0, x1]
                sf_y += [y0, y1]
            else:
                # NaN break so the polyline doesn't join adjacent sectors.
                sector_x += [x0, x1, math.nan]
                sector_y += [y0, y1, math.nan]

        self._startfinish_line.setData(sf_x, sf_y)
        self._sector_lines.setData(sector_x, sector_y)
        self._gate_markers.setData(marker_x, marker_y)

        self._refresh_reference_outline()

    def _refresh_reference_outline(self) -> None:
        """Recompute the gate-layout fingerprint; if it changed, load that
        track's remembered outline (or clear it if none saved)."""
        if self._node_origin_lat is None:
            return
        fp = gate_fingerprint(
            self._node_origin_lat, self._node_origin_lon, self._gates
        )
        if fp == self._fingerprint:
            return
        self._fingerprint = fp

        ref = self._store.load(fp) if fp is not None else None
        if not ref:
            self._reference_trail.setData([], [])
            return
        # Saved as absolute lat/lon; project into the current frame.
        xs = [self._project(lat, lon)[0] for lat, lon in ref]
        ys = [self._project(lat, lon)[1] for lat, lon in ref]
        self._reference_trail.setData(xs, ys)

    def save_current(self) -> None:
        """Persist the live trail under the current gate fingerprint, so a
        later session on the same track reloads it. Call on app close."""
        if self._fingerprint is None or self._origin_lat is None:
            return
        m_per_deg_lon = self._m_per_deg_lon or 1.0
        points = [
            (
                self._origin_lat + n / _M_PER_DEG_LAT,
                self._origin_lon + e / m_per_deg_lon,
            )
            for e, n in zip(self._east, self._north)
        ]
        self._store.save(self._fingerprint, points)
