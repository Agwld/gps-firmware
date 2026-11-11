"""Signals: every message and signal in the CAN database with its live value.

The rows are built from the database, so this screen always shows everything the node sends,
including signals added to the firmware after the other screens were written.
"""

from __future__ import annotations

import time

from PySide6.QtCore import Qt
from PySide6.QtGui import QBrush
from PySide6.QtWidgets import QHeaderView, QLineEdit, QTreeWidget, QTreeWidgetItem

from gps_desktop import theme
from gps_desktop.ui.context import AppContext
from gps_desktop.ui.screens.base import Screen
from gps_desktop.ui.widgets import button, colored, label, screen_layout

REDRAW_INTERVAL = 0.2  # Text that changes faster than this can't be read anyway.
NAME, VALUE, UPDATED, DESCRIPTION = range(4)


class SignalsScreen(Screen):
    title = "Signals"
    nav_label = "Signals"

    def __init__(self, ctx: AppContext) -> None:
        super().__init__(ctx)
        box, self.header = screen_layout(self, "Signals")
        self.tree = QTreeWidget()
        self.tree.setColumnCount(4)
        self.tree.setHeaderLabels(["Message and signal", "Value", "Updated", "Description"])
        self.tree.setAlternatingRowColors(True)
        self.tree.setUniformRowHeights(True)
        header = self.tree.header()
        header.setSectionResizeMode(NAME, QHeaderView.ResizeMode.ResizeToContents)
        header.setSectionResizeMode(VALUE, QHeaderView.ResizeMode.Interactive)
        header.setSectionResizeMode(UPDATED, QHeaderView.ResizeMode.Interactive)
        header.setStretchLastSection(True)
        self.tree.setColumnWidth(VALUE, 150)
        self.tree.setColumnWidth(UPDATED, 110)
        box.addWidget(self.tree, 1)

        self.message_items: dict[str, QTreeWidgetItem] = {}
        self.signal_items: dict[str, QTreeWidgetItem] = {}
        mono = theme.mono_font(10)
        for message in ctx.db.messages:
            top = QTreeWidgetItem(self.tree, [message.name, f"0x{message.frame_id:03X}", "", message.comment])
            font = top.font(NAME)
            font.setBold(True)
            top.setFont(NAME, font)
            top.setToolTip(DESCRIPTION, message.comment)
            self.message_items[message.name] = top
            for signal in message.signals:
                item = QTreeWidgetItem(top, [signal.name, "—", "", signal.comment])
                item.setFont(VALUE, mono)
                item.setToolTip(DESCRIPTION, signal.comment)
                item.setData(NAME, Qt.ItemDataRole.UserRole, signal.cid)
                self.signal_items[signal.cid] = item
        self.tree.expandAll()
        self.tree.itemDoubleClicked.connect(self._double_clicked)
        self.last_draw = 0.0

        self.panel.section("Find")
        self.filter = self.panel.add(QLineEdit())
        self.filter.setPlaceholderText("Signal or message name")
        self.filter.setClearButtonEnabled(True)
        self.filter.textChanged.connect(self._apply_filter)
        self.panel.add(button("Expand all", self.tree.expandAll))
        self.panel.add(button("Collapse all", self.tree.collapseAll))
        self.panel.section("CAN database")
        self.panel.add(label(f"{len(ctx.db.messages)} messages, {len(ctx.db.signals)} signals, read from "
                             f"{ctx.db.path.name}.", "muted", wrap=True))
        self.panel.section("Bus")
        self.bus = self.panel.add(label("", "muted", wrap=True))
        self.panel.note("Double-click a signal to add it to Plots.")
        self.panel.finish()

    def _double_clicked(self, item: QTreeWidgetItem, _column: int) -> None:
        cid = item.data(NAME, Qt.ItemDataRole.UserRole)
        if cid:
            self.ctx.plot(cid)

    def _apply_filter(self, text: str) -> None:
        needle = text.strip().lower()
        for name, top in self.message_items.items():
            shown = 0
            for i in range(top.childCount()):
                child = top.child(i)
                match = not needle or needle in name.lower() or needle in child.text(NAME).lower()
                child.setHidden(not match)
                shown += match
            top.setHidden(shown == 0)

    def restyle(self) -> None:
        self.refresh(force=True)

    def refresh(self, force: bool = False) -> None:
        wall = time.monotonic()
        if not force and wall - self.last_draw < REDRAW_INTERVAL:
            return
        self.last_draw = wall
        data = self.ctx.data
        normal, stale_brush = QBrush(theme.color(theme.TEXT)), QBrush(theme.color(theme.STALE))
        warn = QBrush(theme.color(theme.WARN))
        for name, top in self.message_items.items():
            age = data.age(name)
            stale = name in data.stale
            limit = data.stale_after(name)
            if age is None:
                updated = "Never"
            elif limit is None:
                updated = f"{age:.0f} s ago"  # Sent only when something happens
            elif stale:
                updated = f"Stale for {age:.1f} s"
            else:
                updated = f"{data.rates.get(name, 0.0):.1f} Hz"
            top.setText(UPDATED, updated)
            top.setForeground(UPDATED, warn if stale and age is not None and data.connected else normal)
            brush = stale_brush if stale else normal
            for i in range(top.childCount()):
                child = top.child(i)
                cid = child.data(NAME, Qt.ItemDataRole.UserRole)
                child.setText(VALUE, data.channels[cid].format(data.value(cid)))
                child.setForeground(VALUE, brush)

        total = sum(data.rates.values())
        lines = [f"{total:.0f} frames a second", f"{sum(data.counts.values())} frames received"]
        if data.unknown_frames:
            lines.append(colored(f"{data.unknown_frames} frames not in the database", theme.WARN))
        self.bus.setTextFormat(Qt.TextFormat.RichText)
        self.bus.setText("<br>".join(lines) if data.connected else "Not connected")
        live = len(data.messages) - len(data.stale)
        self.header.summary.setText(f"{live} of {len(data.messages)} messages arriving" if data.connected else "")
