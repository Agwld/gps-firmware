"""Laps: the lap being driven, every completed lap with its sector times, and the event log."""

from __future__ import annotations

import datetime

from PySide6.QtCore import Qt
from PySide6.QtGui import QBrush
from PySide6.QtWidgets import QAbstractItemView, QHBoxLayout, QHeaderView, QTableWidget, QTableWidgetItem

from gps_desktop import theme
from gps_desktop.gps import enums
from gps_desktop.ui.context import AppContext
from gps_desktop.ui.screens.base import Screen
from gps_desktop.ui.widgets import Tile, button, label, screen_layout, section_label


def table(columns: list[str]) -> QTableWidget:
    widget = QTableWidget(0, len(columns))
    widget.setHorizontalHeaderLabels(columns)
    widget.verticalHeader().hide()
    widget.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
    widget.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
    widget.setShowGrid(False)
    widget.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
    widget.horizontalHeader().setDefaultAlignment(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter)
    return widget


def cell(text: str, hex_code: str | None = None, bold: bool = False) -> QTableWidgetItem:
    item = QTableWidgetItem(text)
    if hex_code:
        item.setForeground(QBrush(theme.color(hex_code)))
    if bold:
        font = item.font()
        font.setBold(True)
        item.setFont(font)
    return item


class LapsScreen(Screen):
    title = "Laps"
    nav_label = "Laps"

    def __init__(self, ctx: AppContext) -> None:
        super().__init__(ctx)
        box, self.header = screen_layout(self, "Laps")
        row = QHBoxLayout()
        row.setSpacing(10)
        self.current = Tile(ctx, "Current lap", "Lap_Status.running_time_ms")
        self.sector = Tile(ctx, "Sector", "Lap_Status.sector")
        self.last = Tile(ctx, "Last lap")
        self.best = Tile(ctx, "Best lap")
        self.count = Tile(ctx, "Laps completed", "Lap_Status.lap")
        for tile in (self.current, self.sector, self.last, self.best, self.count):
            row.addWidget(tile)
        box.addLayout(row)

        box.addWidget(section_label("Lap times"))
        self.laps = table(["Lap", "Time", "Gap to best"])
        box.addWidget(self.laps, 3)
        box.addWidget(section_label("Events from the node"))
        self.events = table(["Received", "Event", "Lap", "Time"])
        box.addWidget(self.events, 2)

        self.panel.section("Lap timer")
        self.state = self.panel.add(label("", "muted", wrap=True))
        self.panel.section("History")
        self.panel.add(button("Clear lap history", ctx.data.clear_laps,
                              tooltip="Forget the laps shown here. The node's own lap count is not changed."))
        self.panel.note("Lap and sector times are measured by the node, in GPS time, and arrive as events. "
                        "This screen only lists them, so it has to be connected while they happen.")
        self.panel.note("The fastest lap and the fastest time through each sector are highlighted.")
        self.panel.finish()
        ctx.data.lapsChanged.connect(self._laps_changed)
        self._laps_changed()

    def restyle(self) -> None:
        self._laps_changed()
        self.refresh()

    def _laps_changed(self) -> None:
        data = self.ctx.data
        sectors = max((len(lap.sectors) for lap in data.laps), default=0)
        self.laps.setColumnCount(3 + sectors)
        self.laps.setHorizontalHeaderLabels(["Lap", "Time", "Gap to best"] + [f"Sector {i + 1}" for i in range(sectors)])
        best_sectors = [min((lap.sectors[i] for lap in data.laps if len(lap.sectors) > i), default=None)
                        for i in range(sectors)]
        self.laps.setRowCount(len(data.laps))
        for row, lap in enumerate(reversed(data.laps)):  # Newest first
            is_best = lap.time_ms is not None and lap.time_ms == data.best_lap_ms
            self.laps.setItem(row, 0, cell(str(lap.number)))
            if lap.time_ms is None:
                self.laps.setItem(row, 1, cell("In progress", theme.TEXT_MUTED))
                self.laps.setItem(row, 2, cell(""))
            else:
                self.laps.setItem(row, 1, cell(enums.lap_time(lap.time_ms), theme.OK if is_best else None, is_best))
                gap = (lap.time_ms - data.best_lap_ms) / 1000
                self.laps.setItem(row, 2, cell("Best" if is_best else f"+{gap:.3f}", theme.OK if is_best else None))
            for i in range(sectors):
                if i < len(lap.sectors):
                    fastest = lap.sectors[i] == best_sectors[i]
                    self.laps.setItem(row, 3 + i, cell(enums.lap_time(lap.sectors[i]), theme.ACCENT if fastest else None,
                                                       fastest))
                else:
                    self.laps.setItem(row, 3 + i, cell(""))

        events = data.events[-200:]
        self.events.setRowCount(len(events))
        for row, event in enumerate(reversed(events)):
            received = datetime.datetime.fromtimestamp(event.wall).strftime("%H:%M:%S")
            # A time mark carries GPS time of week rather than a duration.
            time_text = (f"GPS time of week {event.time_ms / 1000:.3f} s" if event.kind == enums.EVENT_MARK
                         else enums.lap_time(event.time_ms))
            lap = "" if event.kind == enums.EVENT_MARK else str(
                event.lap if event.kind == enums.EVENT_LAP else event.lap + 1)
            for column, text in enumerate((received, enums.EVENT_TYPES.get(event.kind, f"Unknown ({event.kind})"),
                                           lap, time_text)):
                self.events.setItem(row, column, cell(text))
        self.refresh()

    def refresh(self) -> None:
        data = self.ctx.data
        flags = data.flags("Lap_Status.flags")
        have = "Lap_Status.flags" in data.values
        running = bool(flags & enums.LAP_RUNNING)
        degraded = bool(flags & enums.LAP_DEGRADED)
        cid = "Lap_Status.running_time_ms"
        self.current.show_value(enums.lap_time(data.value(cid)) if running else "—",
                                self.color(cid, theme.WARN if degraded else None))
        self.sector.show_value(str(int(data.values.get("Lap_Status.sector", 0)) + 1) if running else "—",
                               self.color("Lap_Status.sector"))
        self.count.show_value(str(int(data.values.get("Lap_Status.lap", 0))) if have else "—",
                              self.color("Lap_Status.lap"))
        last = data.last_lap()
        self.last.show_value(enums.lap_time(last.time_ms) if last else "—", None if last else theme.TEXT_FAINT,
                             f"Lap {last.number}" if last else "")
        best = data.best_lap_ms
        self.best.show_value(enums.lap_time(best), theme.OK if best else theme.TEXT_FAINT)

        if not have:
            self.state.setText("No lap timer status yet")
        else:
            lines = ["Timing a lap" if running else
                     "Armed: timing starts when the car next crosses start/finish" if flags & enums.LAP_ARMED else
                     "No start/finish gate. Set one on the Track screen."]
            if degraded:
                lines.append("Degraded: no recent GPS fix or no GPS time lock, so crossings are not timed reliably.")
            self.state.setText("\n\n".join(lines))
        done = sum(1 for lap in data.laps if lap.time_ms is not None)
        self.header.summary.setText(f"{done} lap{'s' if done != 1 else ''} recorded" if done else "")
