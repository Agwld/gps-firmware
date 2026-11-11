"""Track: the map, where the car has been, the timing gates, saved locations and the lines drawn
over them.

Everything here is drawn in metres east and north of one origin. That is the node's own origin
while it has one, so the gates it reports line up exactly; otherwise it is the selected
location, or wherever the map was last left.
"""

from __future__ import annotations

import json
import math
import re
import threading
import urllib.parse
import urllib.request
from pathlib import Path

import numpy as np
import pyqtgraph as pg
from PySide6.QtCore import QSettings, Qt, Signal
from PySide6.QtGui import QKeySequence, QShortcut
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QFileDialog,
    QHBoxLayout,
    QInputDialog,
    QLineEdit,
    QListWidget,
    QListWidgetItem,
    QMenu,
    QMessageBox,
    QPushButton,
    QWidget,
)

from gps_desktop import __version__, theme
from gps_desktop.gps import enums
from gps_desktop.gps.locations import Location, PlannedGate
from gps_desktop.gps.store import from_enu, to_enu
from gps_desktop.gps.tiles import PROVIDERS
from gps_desktop.ui.context import AppContext
from gps_desktop.ui.maplayer import MapLayer
from gps_desktop.ui.screens.base import Screen
from gps_desktop.ui.widgets import Segmented, button, colored, label, screen_layout, style_plot

GATE_HALF_WIDTH_M = 2.0  # LAP_GATE_HALF_WIDTH_M in the firmware's board_config.h
ARROW_M = 3.0  # Length of the arrow showing which way a gate is driven through
HEADING_LINE_M = 4.0
FOLLOW_RANGE_M = 40.0
DEFAULT_ORIGIN = (50.93570, -1.39660)  # Southampton, until a location is chosen
MAP_CHOICES = [None, "map", "satellite"]
NOMINATIM = "https://nominatim.openstreetmap.org/search"
COORDINATES = re.compile(r"^\s*(-?\d+(?:\.\d+)?)\s*[, ]\s*(-?\d+(?:\.\d+)?)\s*$")

# Modes for clicks on the map
VIEW, DRAW, GATE_POINT, GATE_DIRECTION = range(4)


def row(*widgets: QWidget) -> QWidget:
    holder = QWidget()
    line = QHBoxLayout(holder)
    line.setContentsMargins(0, 0, 0, 0)
    line.setSpacing(6)
    for widget in widgets:
        line.addWidget(widget, 1)
    return holder


class TrackScreen(Screen):
    title = "Track"
    nav_label = "Track"
    _found = Signal(object, str)  # A place search finished: (lat, lon) or None, and a message

    def __init__(self, ctx: AppContext) -> None:
        super().__init__(ctx)
        self.settings = QSettings("SUFST", "GPSDesktop")
        self.location: Location | None = None
        self.mode = VIEW
        self.drawing: list[tuple[float, float]] = []  # The line being drawn, as east, north
        self.gate_slot = 0
        self.gate_point: tuple[float, float] | None = None
        self.mouse: tuple[float, float] | None = None
        self.origin_in_use: tuple[float, float] | None = None
        self.items: list = []  # Gates and drawn lines, redrawn whenever they change
        lat = float(self.settings.value("track/lat", DEFAULT_ORIGIN[0]))
        lon = float(self.settings.value("track/lon", DEFAULT_ORIGIN[1]))
        self.fallback_origin = (lat, lon)

        box, self.header = screen_layout(self, "Track")
        self.instruction = QWidget()
        bar = QHBoxLayout(self.instruction)
        bar.setContentsMargins(12, 6, 6, 6)
        self.instruction_text = label("")
        self.instruction_text.setTextFormat(Qt.TextFormat.RichText)
        bar.addWidget(self.instruction_text, 1)
        self.finish_button = button("Finish line", self._finish_line, "primary")
        bar.addWidget(self.finish_button)
        bar.addWidget(button("Cancel", self._cancel_mode))
        self.instruction.hide()
        box.addWidget(self.instruction)

        self.plot = pg.PlotWidget()
        self.plot.setAspectLocked(True)
        self.plot.setLabel("bottom", "East", units="m")
        self.plot.setLabel("left", "North", units="m")
        self.plot.setMenuEnabled(False)
        self.plot.hideButtons()
        box.addWidget(self.plot, 1)
        self.attribution = label("", "faint")
        self.attribution.setAlignment(Qt.AlignmentFlag.AlignRight)
        box.addWidget(self.attribution)

        self.map = MapLayer(self.plot, ctx.tiles)
        self.trail = self.plot.plot([], [], antialias=True)
        self.preview = self.plot.plot([], [], antialias=True)
        self.heading = self.plot.plot([], [], antialias=True)
        self.car = pg.ScatterPlotItem(size=14)
        self.car.setZValue(20)
        self.plot.addItem(self.car)
        self.plot.scene().sigMouseClicked.connect(self._clicked)
        self.plot.scene().sigMouseMoved.connect(self._moved)

        self._build_panel()

        for keys, action in (("Escape", self._cancel_mode), ("Backspace", self._undo_point),
                             ("Return", self._finish_line), ("Enter", self._finish_line)):
            shortcut = QShortcut(QKeySequence(keys), self)
            shortcut.setContext(Qt.ShortcutContext.WidgetWithChildrenShortcut)
            shortcut.activated.connect(action)

        # The node's origin arriving comes with gatesChanged, as gates are relative to it.
        ctx.data.gatesChanged.connect(self._connection_changed)
        ctx.data.connectionChanged.connect(self._connection_changed)
        self.plot.getViewBox().setRange(xRange=(-150, 150), yRange=(-150, 150), padding=0)
        ctx.locations.changed.connect(self._fill_locations)
        ctx.uploadFinished.connect(self._upload_finished)
        ctx.tiles.failed.connect(ctx.message)
        self._found.connect(self._on_found)
        self._fill_locations()
        self.map.set_provider(MAP_CHOICES[self.map_choice.current()])
        self.restyle()
        self._connection_changed()
        if ctx.locations.error:
            ctx.message.emit(ctx.locations.error)

    # Panel

    def _build_panel(self) -> None:
        panel = self.panel
        panel.section("Location")
        self.location_box = panel.add(QComboBox())
        self.location_box.currentIndexChanged.connect(self._location_chosen)
        self.save_location = button("Save as a location…", self._save_location,
                                    tooltip="Name the place the map is showing, to keep its lines and gates")
        more = QPushButton("More")
        more.setCursor(Qt.CursorShape.PointingHandCursor)
        menu = QMenu(more)
        self.rename_action = menu.addAction("Rename this location…", self._rename_location)
        self.move_action = menu.addAction("Centre this location on the map view", self._recentre_location)
        self.delete_action = menu.addAction("Delete this location…", self._delete_location)
        menu.addSeparator()
        menu.addAction("Export all locations…", self._export)
        menu.addAction("Import locations…", self._import)
        more.setMenu(menu)
        panel.add(row(self.save_location, more))
        self.search = QLineEdit()
        self.search.setPlaceholderText("Find a place, or paste latitude, longitude")
        self.search.returnPressed.connect(self._find_place)
        self.search.setClearButtonEnabled(True)
        panel.add(self.search)

        panel.section("Map")
        saved = self.settings.value("track/map", "satellite")
        index = MAP_CHOICES.index(saved) if saved in MAP_CHOICES else 0
        self.map_choice = panel.add(Segmented(["None", "Map", "Satellite"], current=index))
        self.map_choice.changed.connect(self._map_changed)

        panel.section("Draw")
        self.draw_button = panel.add(button("Draw a track line", self._start_drawing,
                                            tooltip="Click points along the track. Double-click or press Enter to finish."))
        self.delete_line = button("Delete last line", self._delete_last_line)
        self.delete_lines = button("Delete all", self._delete_all_lines)
        panel.add(row(self.delete_line, self.delete_lines))

        panel.section("Gates")
        self.place_buttons = [
            button("Place start/finish", lambda: self._start_gate(0), "primary",
                   "Click where the line goes on the map, then click the direction the car drives through it"),
            button("Place a sector", lambda: self._start_gate(enums.SLOT_NEXT_SECTOR),
                   tooltip="Click where the gate goes on the map, then click the direction the car drives through it"),
        ]
        panel.add(row(*self.place_buttons))
        self.car_buttons = [
            button("Start/finish at the car", lambda: self._gate_at_car(0),
                   tooltip="Place the start/finish line where the car is now, like a long press of the lap button"),
            button("Sector at the car", lambda: self._gate_at_car(enums.SLOT_NEXT_SECTOR),
                   tooltip="Place the next sector gate where the car is now, like a short press of the lap button"),
        ]
        panel.add(row(*self.car_buttons))
        self.gate_list = panel.add(QListWidget())
        self.gate_list.setFixedHeight(200)
        self.gate_list.itemSelectionChanged.connect(self._update_buttons)
        self.clear_one = button("Clear selected", self._clear_selected)
        self.clear_all = button("Clear all", self._clear_all, "danger")
        panel.add(row(self.clear_one, self.clear_all))

        panel.section("Saving to the node")
        self.sync_text = panel.add(label("", "muted", wrap=True))
        self.sync_text.setTextFormat(Qt.TextFormat.RichText)
        self.send_button = panel.add(button("Send this layout to the node", self._send_layout, "primary",
                                            "Replace the node's gates with this location's, then check the node has them"))
        self.copy_button = panel.add(button("Save the node's gates to this location", self._copy_from_node,
                                            tooltip="Replace this location's gate layout with the gates the node has now"))
        self.flash_text = panel.add(label("", "muted", wrap=True))
        self.flash_text.setTextFormat(Qt.TextFormat.RichText)

        panel.section("View")
        self.follow = panel.add(QCheckBox("Follow the car"))
        self.follow.toggled.connect(lambda follow: None if follow else self._fit())
        panel.add(row(button("Fit to track", self._fit), button("Clear trail", self.ctx.data.clear_trail)))
        self.readout = panel.add(label("", "muted", wrap=True))
        panel.finish()

    # Origin and coordinates

    def view_origin(self) -> tuple[float, float]:
        return self.ctx.node_origin() or (
            (self.location.lat, self.location.lon) if self.location else self.fallback_origin)

    def to_view(self, lat, lon):
        return to_enu(lat, lon, self.origin_in_use or self.view_origin())

    def to_latlon(self, east: float, north: float) -> tuple[float, float]:
        return from_enu(east, north, self.origin_in_use or self.view_origin())

    def _check_origin(self) -> None:
        """Move everything to a new origin, keeping the same place in the middle of the view."""
        origin = self.view_origin()
        if origin == self.origin_in_use:
            return
        box = self.plot.getViewBox()
        (x0, x1), (y0, y1) = box.viewRange()
        centre = self.to_latlon((x0 + x1) / 2, (y0 + y1) / 2) if self.origin_in_use else None
        self.origin_in_use = origin
        self.map.set_origin(origin)
        self._cancel_mode()
        if centre is not None:
            east, north = self.to_view(*centre)
            half_x, half_y = (x1 - x0) / 2, (y1 - y0) / 2
            box.setRange(xRange=(east - half_x, east + half_x), yRange=(north - half_y, north + half_y), padding=0)
        self._draw_overlays()

    def _centre_on(self, lat: float, lon: float, span_m: float = 300.0) -> None:
        self.follow.setChecked(False)
        east, north = self.to_view(lat, lon)
        self.plot.getViewBox().setRange(xRange=(east - span_m / 2, east + span_m / 2),
                                        yRange=(north - span_m / 2, north + span_m / 2), padding=0)

    def _view_centre(self) -> tuple[float, float]:
        (x0, x1), (y0, y1) = self.plot.getViewBox().viewRange()
        return self.to_latlon((x0 + x1) / 2, (y0 + y1) / 2)

    # Locations

    def _fill_locations(self) -> None:
        current = self.location.name if self.location else None
        self.location_box.blockSignals(True)
        self.location_box.clear()
        self.location_box.addItem("No location")
        self.location_box.addItems(self.ctx.locations.names())
        self.location = self.ctx.locations.get(current) if current else None
        self.location_box.setCurrentIndex(self.location_box.findText(current) if self.location else 0)
        self.location_box.blockSignals(False)
        self._draw_overlays()

    def _location_chosen(self, index: int) -> None:
        name = self.location_box.itemText(index) if index > 0 else None
        self.location = self.ctx.locations.get(name) if name else None
        self._cancel_mode()
        self._check_origin()
        if self.location is not None:
            self._centre_on(self.location.lat, self.location.lon)
        self._draw_overlays()

    def _select(self, name: str) -> None:
        self.location_box.setCurrentIndex(self.location_box.findText(name))

    def _ask_name(self, title: str, text: str = "") -> str | None:
        name, ok = QInputDialog.getText(self, title, "Name:", text=text)
        name = name.strip()
        if not ok or not name:
            return None
        if self.ctx.locations.get(name) is not None and name != text:
            QMessageBox.warning(self, title, f"There is already a location called {name}.")
            return None
        return name

    def _save_location(self) -> bool:
        name = self._ask_name("Save as a location")
        if name is None:
            return False
        lat, lon = self._view_centre()
        self.ctx.locations.add(Location(name, lat, lon))
        self._select(name)
        self.ctx.message.emit(f"Saved {name}")
        return True

    def _need_location(self, what: str) -> bool:
        """Lines and gate layouts belong to a location. Offer to make one if none is chosen."""
        if self.location is not None:
            return True
        answer = QMessageBox.question(self, "Save a location first",
                                      f"{what} are kept with a saved location. Save the place on the map now?")
        return answer == QMessageBox.StandardButton.Yes and self._save_location()

    def _rename_location(self) -> None:
        if self.location is not None:
            name = self._ask_name("Rename location", self.location.name)
            if name and name != self.location.name:
                self.ctx.locations.rename(self.location.name, name)
                self._select(name)

    def _recentre_location(self) -> None:
        if self.location is not None:
            self.location.lat, self.location.lon = self._view_centre()
            self.ctx.locations.save()
            self.ctx.message.emit(f"{self.location.name} now opens centred here")

    def _delete_location(self) -> None:
        if self.location is None:
            return
        answer = QMessageBox.question(self, "Delete location",
                                      f"Delete {self.location.name}, with its lines and gate layout? "
                                      "Gates already on the node are not changed.")
        if answer == QMessageBox.StandardButton.Yes:
            self.ctx.locations.remove(self.location.name)
            self.location_box.setCurrentIndex(0)

    def _export(self) -> None:
        path, _ = QFileDialog.getSaveFileName(self, "Export locations", "gps-locations.json", "Locations (*.json)")
        if path:
            self.ctx.locations.write(Path(path), self.ctx.locations.items)
            self.ctx.message.emit(f"Exported {len(self.ctx.locations.items)} locations")

    def _import(self) -> None:
        path, _ = QFileDialog.getOpenFileName(self, "Import locations", "", "Locations (*.json)")
        if not path:
            return
        try:
            count = self.ctx.locations.import_file(Path(path))
        except (OSError, ValueError, KeyError, TypeError) as error:
            QMessageBox.warning(self, "Import locations", f"Could not read that file: {error}")
            return
        self.ctx.message.emit(f"Imported {count} locations")

    def _find_place(self) -> None:
        text = self.search.text().strip()
        if not text:
            return
        match = COORDINATES.match(text)
        if match:
            self._on_found((float(match.group(1)), float(match.group(2))), "")
            return
        self.ctx.message.emit(f"Looking for {text}…")
        user_agent = f"GPSDesktop/{__version__}"

        def work() -> None:
            url = f"{NOMINATIM}?{urllib.parse.urlencode({'q': text, 'format': 'json', 'limit': 1})}"
            try:
                request = urllib.request.Request(url, headers={"User-Agent": user_agent})
                from gps_desktop.gps.tiles import _ssl_context

                with urllib.request.urlopen(request, timeout=15, context=_ssl_context()) as response:
                    results = json.load(response)
            except Exception as error:  # Offline is common at a track; say so rather than fail.
                self._found.emit(None, f"Could not search for places: {error}")
                return
            if not results:
                self._found.emit(None, f"Nothing found for {text}")
            else:
                self._found.emit((float(results[0]["lat"]), float(results[0]["lon"])), "")

        threading.Thread(target=work, daemon=True).start()

    def _on_found(self, place: tuple[float, float] | None, message: str) -> None:
        if place is None:
            self.ctx.message.emit(message)
            return
        if self.ctx.node_origin() is None and self.location is None:
            self.fallback_origin = place
            self._check_origin()
        self._centre_on(*place)

    # Map

    def _map_changed(self, index: int) -> None:
        self.settings.setValue("track/map", MAP_CHOICES[index] or "none")
        self.map.set_provider(MAP_CHOICES[index])
        self.attribution.setText(self.map.attribution())

    # Clicking on the map

    def _plot_point(self, scene_pos) -> tuple[float, float] | None:
        box = self.plot.getViewBox()
        if not box.sceneBoundingRect().contains(scene_pos):
            return None
        point = box.mapSceneToView(scene_pos)
        return point.x(), point.y()

    def _clicked(self, event) -> None:
        if self.mode == VIEW or event.button() != Qt.MouseButton.LeftButton:
            return
        point = self._plot_point(event.scenePos())
        if point is None:
            return
        event.accept()
        if self.mode == DRAW:
            if event.double():
                self._finish_line()
            else:
                self.drawing.append(point)
        elif self.mode == GATE_POINT:
            self.gate_point = point
            self.mode = GATE_DIRECTION
        elif self.mode == GATE_DIRECTION and self.gate_point is not None:
            de, dn = point[0] - self.gate_point[0], point[1] - self.gate_point[1]
            if math.hypot(de, dn) < 0.5:
                return  # Too close to tell a direction
            self._place_gate(self.gate_point, math.degrees(math.atan2(dn, de)) % 360)
            return
        self._show_mode()

    def _moved(self, scene_pos) -> None:
        self.mouse = self._plot_point(scene_pos)
        if self.mode != VIEW:
            self._draw_preview()

    def _show_mode(self) -> None:
        texts = {
            DRAW: "<b>Drawing a track line.</b> Click points along the track. Double-click or press Enter to "
                  "finish, Backspace to undo a point.",
            GATE_POINT: f"<b>Placing {self._slot_name()}.</b> Click where the line crosses the track.",
            GATE_DIRECTION: f"<b>Placing {self._slot_name()}.</b> Now click in the direction the car drives "
                            "through it.",
        }
        self.instruction.setVisible(self.mode != VIEW)
        self.instruction_text.setText(texts.get(self.mode, ""))
        self.finish_button.setVisible(self.mode == DRAW)
        c = theme.color(theme.ACCENT)
        self.instruction.setStyleSheet(f"background: rgba({c.red()},{c.green()},{c.blue()},30); border-radius: 6px;")
        self.plot.setCursor(Qt.CursorShape.CrossCursor if self.mode != VIEW else Qt.CursorShape.ArrowCursor)
        self._draw_preview()

    def _cancel_mode(self) -> None:
        self.mode = VIEW
        self.drawing = []
        self.gate_point = None
        self._show_mode()

    def _start_drawing(self) -> None:
        if self._need_location("Track lines"):
            self._cancel_mode()
            self.mode = DRAW
            self._show_mode()

    def _undo_point(self) -> None:
        if self.mode == DRAW and self.drawing:
            self.drawing.pop()
            self._draw_preview()

    def _finish_line(self) -> None:
        if self.mode != DRAW:
            return
        points = [p for i, p in enumerate(self.drawing) if i == 0 or p != self.drawing[i - 1]]
        if len(points) >= 2 and self.location is not None:
            self.location.lines.append([list(self.to_latlon(e, n)) for e, n in points])
            self.ctx.locations.save()
        self._cancel_mode()

    def _delete_last_line(self) -> None:
        if self.location is not None and self.location.lines:
            self.location.lines.pop()
            self.ctx.locations.save()

    def _delete_all_lines(self) -> None:
        if self.location is None or not self.location.lines:
            return
        answer = QMessageBox.question(self, "Delete lines", f"Delete every line drawn at {self.location.name}?")
        if answer == QMessageBox.StandardButton.Yes:
            self.location.lines.clear()
            self.ctx.locations.save()

    # Gates

    def _slot_name(self) -> str:
        return "the start/finish line" if self.gate_slot == 0 else "a sector gate"

    def _start_gate(self, slot: int) -> None:
        if not self.ctx.can_place_gates() and not self._need_location("Gate layouts"):
            return
        self._cancel_mode()
        self.gate_slot = slot
        self.mode = GATE_POINT
        self._show_mode()

    def _next_sector(self) -> int:
        if self.location is not None:
            return self.location.next_sector()
        return next((s for s in range(1, enums.GATE_COUNT) if s not in self.ctx.data.gates), enums.GATE_COUNT - 1)

    def _place_gate(self, point: tuple[float, float], heading: float) -> None:
        slot = self._next_sector() if self.gate_slot == enums.SLOT_NEXT_SECTOR else self.gate_slot
        lat, lon = self.to_latlon(*point)
        if self.location is not None:
            self.location.set_gate(PlannedGate(slot, lat, lon, heading))
            self.ctx.locations.save()
        sent = self.ctx.set_gate_at(slot, lat, lon, heading, quiet=True)
        where = " and sent to the node" if sent else ""
        self.ctx.message.emit(f"{enums.gate_name(slot)} saved to {self.location.name}{where}" if self.location
                              else f"{enums.gate_name(slot)} sent to the node")
        self._cancel_mode()
        self._draw_overlays()

    def _gate_at_car(self, slot: int) -> None:
        self.ctx.command(enums.CMD_GATE_SET, slot,
                         done="Start/finish set at the car" if slot == 0 else "Sector gate set at the car")

    def _selected_slot(self) -> int | None:
        item = self.gate_list.currentItem()
        return None if item is None else item.data(Qt.ItemDataRole.UserRole)

    def _clear_selected(self) -> None:
        slot = self._selected_slot()
        if slot is None:
            return
        if self.location is not None:
            self.location.clear_gate(slot)
            self.ctx.locations.save()
        if self.ctx.connected and slot in self.ctx.data.gates:
            self.ctx.command(enums.CMD_GATE_CLEAR, slot, done=f"{enums.gate_name(slot)} cleared")
        self._draw_overlays()

    def _clear_all(self) -> None:
        where = [w for w, on in ((f"the {self.location.name} layout", self.location and self.location.gates),
                                 ("the node", self.ctx.connected and self.ctx.data.gates)) if on]
        if not where:
            return
        answer = QMessageBox.question(self, "Clear all gates", f"Remove every gate from {' and '.join(where)}?")
        if answer != QMessageBox.StandardButton.Yes:
            return
        if self.location is not None:
            self.location.gates.clear()
            self.ctx.locations.save()
        if self.ctx.connected:
            self.ctx.command(enums.CMD_GATE_CLEAR, enums.SLOT_ALL, done="All gates cleared")
        self._draw_overlays()

    def _send_layout(self) -> None:
        if self.location is None:
            return
        node_has = len(self.ctx.data.gates)
        if node_has and not self.location.gates:
            answer = QMessageBox.question(self, "Send layout", f"{self.location.name} has no gates. Clear the "
                                          f"node's {node_has} gate{'s' if node_has != 1 else ''}?")
            if answer != QMessageBox.StandardButton.Yes:
                return
        if self.ctx.send_layout(self.location):
            self.send_button.setEnabled(False)
            self.sync_text.setText(colored("Sending, then checking what the node reports back…", theme.TEXT_MUTED))

    def _upload_finished(self, ok: bool, text: str) -> None:
        self.ctx.message.emit(text)
        if not ok:
            QMessageBox.warning(self, "Send layout", text)
        self._update_buttons()
        self._draw_overlays()

    def _copy_from_node(self) -> None:
        if self.location is None:
            return
        if self.location.gates:
            answer = QMessageBox.question(self, "Save the node's gates",
                                          f"Replace the gate layout saved for {self.location.name} with the "
                                          "gates the node has now?")
            if answer != QMessageBox.StandardButton.Yes:
                return
        count = self.ctx.copy_node_gates(self.location)
        self.ctx.message.emit(f"Saved {count} gates to {self.location.name}")

    # State

    def _connection_changed(self) -> None:
        origin = self.ctx.node_origin()
        if origin is not None and self.location is None:
            # Arriving at a known track: open its location automatically.
            nearby = self.ctx.locations.nearest(*origin)
            if nearby is not None:
                self._select(nearby.name)
                self.ctx.message.emit(f"At {nearby.name}")
        self._check_origin()
        self._draw_overlays()

    def _update_buttons(self) -> None:
        connected = self.ctx.connected
        have_location = self.location is not None
        for widget in self.car_buttons:
            widget.setEnabled(connected)
        for widget in self.place_buttons:
            widget.setEnabled(have_location or self.ctx.can_place_gates())
        self.rename_action.setEnabled(have_location)
        self.move_action.setEnabled(have_location)
        self.delete_action.setEnabled(have_location)
        self.delete_line.setEnabled(have_location and bool(self.location.lines))
        self.delete_lines.setEnabled(have_location and bool(self.location.lines))
        self.clear_one.setEnabled(self._selected_slot() is not None)
        self.clear_all.setEnabled(bool(self.ctx.data.gates) or (have_location and bool(self.location.gates)))
        self.send_button.setEnabled(have_location and self.ctx.can_place_gates() and self.ctx._upload is None)
        self.copy_button.setEnabled(have_location and connected and self.ctx.node_origin() is not None)
        self._update_sync()

    def _update_sync(self) -> None:
        if self.location is None:
            text = "Choose or save a location to keep a gate layout for it, and send it to the node in one go."
        elif not self.ctx.connected:
            count = len(self.location.gates)
            text = f"{count} gate{'s' if count != 1 else ''} saved for {self.location.name}. Connect to send them."
        elif self.ctx.node_origin() is None:
            text = colored("Waiting for the node to have a GPS position.", theme.WARN)
        else:
            differences = self.ctx.layout_differences(self.location)
            text = (colored(f"The node has the {self.location.name} layout.", theme.OK) if not differences else
                    colored("The node's gates differ from this layout:", theme.WARN) + "<br>" +
                    "<br>".join(f"· {d}" for d in differences))
        self.sync_text.setText(text)

        bits = self.ctx.data.flags("GPS_Status.fault_bits")
        if not self.ctx.connected or "GPS_Status.fault_bits" not in self.ctx.data.values:
            flash = "The node saves gates to its flash as soon as they are set."
        elif bits & enums.FLASH_ERROR:
            flash = colored("The node's last flash write failed. Gates may not survive a power cycle.", theme.FAULT)
        elif bits & enums.FLASH_UNSAVED:
            flash = colored("Some changes are not in the node's flash yet. They are written when the car "
                            "next stops.", theme.WARN)
        else:
            flash = colored("All gates are saved in the node's flash.", theme.OK)
        self.flash_text.setText(flash)

    # Drawing

    def restyle(self) -> None:
        self.plot.setBackground(theme.BG)
        style_plot(self.plot.getPlotItem())
        self.trail.setPen(pg.mkPen(theme.SERIES[0], width=2))
        self.heading.setPen(pg.mkPen(theme.TEXT, width=2))
        self.car.setBrush(pg.mkBrush(theme.TEXT))
        self.car.setPen(pg.mkPen(theme.BG, width=2))
        self.attribution.setText(self.map.attribution())
        self._draw_overlays()
        self._show_mode()
        self.refresh()

    def _gate_items(self, east: float, north: float, heading: float, hex_code: str, text: str, solid: bool) -> list:
        h = math.radians(heading)
        along_e, along_n = math.cos(h), math.sin(h)
        across_e, across_n = -along_n, along_e
        style = Qt.PenStyle.SolidLine if solid else Qt.PenStyle.DashLine
        line = self.plot.plot([east - across_e * GATE_HALF_WIDTH_M, east + across_e * GATE_HALF_WIDTH_M],
                              [north - across_n * GATE_HALF_WIDTH_M, north + across_n * GATE_HALF_WIDTH_M],
                              pen=pg.mkPen(hex_code, width=4 if solid else 2, style=style), antialias=True)
        tip_e, tip_n = east + along_e * ARROW_M, north + along_n * ARROW_M
        arrow = self.plot.plot([east, tip_e, tip_e - along_e * 0.9 + across_e * 0.6, tip_e,
                                tip_e - along_e * 0.9 - across_e * 0.6],
                               [north, tip_n, tip_n - along_n * 0.9 + across_n * 0.6, tip_n,
                                tip_n - along_n * 0.9 - across_n * 0.6],
                               pen=pg.mkPen(hex_code, width=2, style=style), antialias=True)
        # A backing so the label reads on satellite imagery as well as on a plain background.
        label_item = pg.TextItem(text, color=hex_code, anchor=(0.5, 1.4), fill=pg.mkBrush(theme.color(theme.BG, 210)))
        label_item.setPos(east - along_e * 1.0, north - along_n * 1.0)
        for item in (line, arrow, label_item):
            item.setZValue(10)
        self.plot.addItem(label_item)
        return [line, arrow, label_item]

    def _gate_rows(self) -> list[tuple[int, PlannedGate | None, tuple | None, bool]]:
        """For each slot in use: the planned gate, the node's gate, and whether they match."""
        rows = []
        differences = set()
        if self.location is not None and self.ctx.node_origin() is not None:
            differences = {d.split(" is ")[0] for d in self.ctx.layout_differences(self.location)}
        for slot in range(enums.GATE_COUNT):
            planned = self.location.gate(slot) if self.location else None
            actual = self.ctx.node_gate_latlon(slot)
            if planned or actual:
                rows.append((slot, planned, actual, enums.gate_name(slot) not in differences))
        return rows

    def _draw_overlays(self) -> None:
        """Redraw the gates and the drawn lines, and refill the gate list."""
        if self.origin_in_use is None:
            return
        for item in self.items:
            self.plot.removeItem(item)
        self.items = []
        if self.location is not None:
            for line in self.location.lines:
                points = np.array(line)
                east, north = self.to_view(points[:, 0], points[:, 1])
                item = self.plot.plot(east, north, pen=pg.mkPen(theme.SERIES[1], width=3), antialias=True)
                item.setZValue(5)
                self.items.append(item)

        selected = self._selected_slot()
        self.gate_list.blockSignals(True)
        self.gate_list.clear()
        for slot, planned, actual, matches in self._gate_rows():
            hex_code = theme.OK if slot == 0 else theme.WARN
            if actual is not None:
                east, north = self.to_view(actual[0], actual[1])
                self.items += self._gate_items(float(east), float(north), actual[2], hex_code,
                                               enums.gate_label(slot), True)
            if planned is not None and (actual is None or not matches):
                east, north = self.to_view(planned.lat, planned.lon)
                self.items += self._gate_items(float(east), float(north), planned.heading, theme.TEXT_MUTED,
                                               enums.gate_label(slot), False)
            heading = (actual[2] if actual else planned.heading)
            if actual and planned:
                status = "On the node and in the layout" if matches else "On the node, but not where the layout has it"
            elif actual:
                status = "On the node" + (", not in the layout" if self.location else "")
            else:
                status = "In the layout, not on the node" if self.ctx.connected else "In the layout"
            item = QListWidgetItem(f"{enums.gate_name(slot)} · heading {enums.bearing(heading):.0f}°\n{status}")
            item.setData(Qt.ItemDataRole.UserRole, slot)
            self.gate_list.addItem(item)
            if slot == selected:
                self.gate_list.setCurrentItem(item)
        self.gate_list.blockSignals(False)
        self._update_buttons()

    def _draw_preview(self) -> None:
        pen = pg.mkPen(theme.SERIES[1], width=2, style=Qt.PenStyle.DashLine)
        points: list[tuple[float, float]] = []
        if self.mode == DRAW:
            points = list(self.drawing) + ([self.mouse] if self.mouse and self.drawing else [])
        elif self.mode == GATE_DIRECTION and self.gate_point and self.mouse:
            points = [self.gate_point, self.mouse]
            pen = pg.mkPen(theme.OK if self.gate_slot == 0 else theme.WARN, width=2, style=Qt.PenStyle.DashLine)
        self.preview.setPen(pen)
        self.preview.setSymbol("o" if self.mode == DRAW else None)
        self.preview.setSymbolSize(6)
        self.preview.setSymbolBrush(pg.mkBrush(theme.SERIES[1]))
        if points:
            self.preview.setData([p[0] for p in points], [p[1] for p in points])
        else:
            self.preview.setData([], [])

    def _fit(self) -> None:
        self.follow.setChecked(False)
        east, north = self._trail()
        xs = list(east)
        ys = list(north)
        for item in self.items:
            if isinstance(item, pg.PlotDataItem):
                x, y = item.getData()
                if x is not None:
                    xs += list(x)
                    ys += list(y)
        if not xs:
            self.plot.getViewBox().setRange(xRange=(-150, 150), yRange=(-150, 150), padding=0)
            return
        self.plot.getViewBox().setRange(xRange=(min(xs), max(xs)), yRange=(min(ys), max(ys)), padding=0.08)

    def _trail(self):
        lat, lon = self.ctx.data.trail.arrays()
        if not len(lat) or self.origin_in_use is None:
            return np.zeros(0), np.zeros(0)
        return self.to_view(lat, lon)

    def refresh(self) -> None:
        self._check_origin()
        data = self.ctx.data
        east, north = self._trail()
        self.trail.setData(east, north)
        lat, lon = data.values.get("GPS_Position.lat_deg"), data.values.get("GPS_Position.lon_deg")
        if lat is None or (lat == 0.0 and lon == 0.0):
            self.car.setData([], [])
            self.heading.setData([], [])
            self.readout.setText("No position from the node yet" if self.ctx.connected else "")
        else:
            e, n = (float(v) for v in self.to_view(lat, lon))
            self.car.setData([e], [n])
            yaw = data.value("GPS_Attitude.yaw_deg")  # A compass bearing
            if yaw is None:
                self.heading.setData([], [])
            else:
                yaw = math.radians(yaw)
                self.heading.setData([e, e + math.sin(yaw) * HEADING_LINE_M], [n, n + math.cos(yaw) * HEADING_LINE_M])
            if self.follow.isChecked():
                self.plot.setRange(xRange=(e - FOLLOW_RANGE_M, e + FOLLOW_RANGE_M),
                                   yRange=(n - FOLLOW_RANGE_M, n + FOLLOW_RANGE_M), padding=0)
            speed = data.values.get("GPS_Velocity.speed_mps", 0.0)
            self.readout.setText(f"{lat:.6f}°, {lon:.6f}° · {speed * 3.6:.1f} km/h")
        self._update_sync()

        parts = []
        if self.location is not None:
            parts.append(self.location.name)
        if self.ctx.node_origin() is None and self.ctx.connected:
            parts.append(colored("the node has no GPS position yet", theme.WARN))
        self.header.summary.setText(" · ".join(parts))

    def hideEvent(self, event) -> None:
        super().hideEvent(event)
        if self.ctx.node_origin() is None and self.origin_in_use is not None:
            lat, lon = self._view_centre()
            self.settings.setValue("track/lat", lat)
            self.settings.setValue("track/lon", lon)
