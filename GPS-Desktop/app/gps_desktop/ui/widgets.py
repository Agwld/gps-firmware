"""Small building blocks shared by the screens."""

from __future__ import annotations

from collections.abc import Callable

import pyqtgraph as pg
from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QMouseEvent
from PySide6.QtWidgets import (
    QButtonGroup,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)

from gps_desktop import theme
from gps_desktop.ui.context import AppContext

VALUE_STYLE = "font-size: 20px; font-weight: 600;"


def label(text: str = "", name: str = "", wrap: bool = False) -> QLabel:
    widget = QLabel(text)
    if name:
        widget.setObjectName(name)
    widget.setWordWrap(wrap)
    return widget


def section_label(text: str) -> QLabel:
    return label(text.upper(), "sectionLabel")


def button(text: str, on_click: Callable | None = None, name: str = "", tooltip: str = "") -> QPushButton:
    widget = QPushButton(text)
    if name:
        widget.setObjectName(name)
    if on_click is not None:
        widget.clicked.connect(lambda _checked=False: on_click())
    if tooltip:
        widget.setToolTip(tooltip)
    widget.setCursor(Qt.CursorShape.PointingHandCursor)
    return widget


class Panel(QWidget):
    """The contents of the left configuration panel for one screen."""

    def __init__(self, title: str) -> None:
        super().__init__()
        self.box = QVBoxLayout(self)
        self.box.setContentsMargins(16, 16, 16, 16)
        self.box.setSpacing(8)
        self.box.addWidget(label(title, "panelTitle"))

    def section(self, title: str) -> None:
        self.box.addWidget(section_label(title))

    def add(self, widget: QWidget) -> QWidget:
        self.box.addWidget(widget)
        return widget

    def note(self, text: str) -> QLabel:
        widget = label(text, "faint", wrap=True)
        self.box.addWidget(widget)
        return widget

    def finish(self) -> None:
        self.box.addStretch(1)


class Card(QFrame):
    def __init__(self, title: str = "", parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setObjectName("card")
        self.box = QVBoxLayout(self)
        self.box.setContentsMargins(14, 12, 14, 14)
        self.box.setSpacing(8)
        if title:
            self.box.addWidget(label(title, "cardTitle"))


class ScreenHeader(QWidget):
    def __init__(self, title: str) -> None:
        super().__init__()
        row = QHBoxLayout(self)
        row.setContentsMargins(0, 0, 0, 0)
        row.addWidget(label(title, "screenTitle"))
        row.addSpacing(16)
        self.summary = label("", "screenSummary")
        self.summary.setTextFormat(Qt.TextFormat.RichText)
        row.addWidget(self.summary, 1)
        self.right = QHBoxLayout()
        row.addLayout(self.right)


def screen_layout(widget: QWidget, title: str) -> tuple[QVBoxLayout, ScreenHeader]:
    box = QVBoxLayout(widget)
    box.setContentsMargins(20, 16, 20, 16)
    box.setSpacing(12)
    header = ScreenHeader(title)
    box.addWidget(header)
    return box, header


def colored(text: str, hex_code: str) -> str:
    return f"<span style='color:{hex_code}'>{text}</span>"


class Badge(QLabel):
    """A pill showing a named state in its state colour."""

    def __init__(self, text: str = "—", compact: bool = False) -> None:
        super().__init__(text)
        self.compact = compact  # Smaller, for a grid of flags
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.set_state(text, theme.TEXT_FAINT)

    def set_state(self, text: str, hex_code: str) -> None:
        if (text, hex_code) == getattr(self, "_shown", None):
            return  # Restyling is slow, and most refreshes change nothing.
        self._shown = (text, hex_code)
        self.setText(text)
        c = theme.color(hex_code)
        size = "border-radius: 9px; padding: 2px 9px; font-size: 12px;" if self.compact else \
            "border-radius: 11px; padding: 3px 12px;"
        self.setStyleSheet(
            f"QLabel {{ color: {hex_code}; background: rgba({c.red()},{c.green()},{c.blue()},38);"
            f" border: 1px solid rgba({c.red()},{c.green()},{c.blue()},110); {size} font-weight: 600; }}"
        )


class Tile(QFrame):
    """One live value: caption, big value or badge, and a smaller line underneath.

    Give it a signal id and hovering shows where the value comes from; clicking adds it to Plots.
    """

    def __init__(self, ctx: AppContext, caption: str, cid: str | None = None, badge: bool = False) -> None:
        super().__init__()
        self.ctx = ctx
        self.cid = cid
        self.setObjectName("card")
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        box = QVBoxLayout(self)
        box.setContentsMargins(14, 10, 14, 12)
        box.setSpacing(2)
        box.addWidget(label(caption, "tileCaption"))
        self.badge = Badge() if badge else None
        self.value = QLabel("—")
        if self.badge is not None:
            row = QHBoxLayout()
            row.addWidget(self.badge)
            row.addStretch(1)
            box.addSpacing(4)
            box.addLayout(row)
        else:
            self.value.setStyleSheet(VALUE_STYLE)
            box.addWidget(self.value)
        self.sub = label(" ", "tileSub")  # Always present, so every tile in a row is the same height
        box.addWidget(self.sub)
        self._shown: tuple | None = None
        info = ctx.data.channels.get(cid) if cid else None
        if info is None:
            self.cid = None  # A signal this database does not have
        else:
            self.setCursor(Qt.CursorShape.PointingHandCursor)
            self.setToolTip(f"<b>{info.message}.{info.name}</b><br>{info.comment or ctx.data.messages[info.message].comment}"
                            "<br><i>Click to plot</i>")

    def show_value(self, text: str, hex_code: str | None = None, sub: str | None = None) -> None:
        hex_code = hex_code or theme.TEXT
        if (text, hex_code, sub) == self._shown:
            return
        self._shown = (text, hex_code, sub)
        if self.badge is not None:
            self.badge.set_state(text, hex_code)
        else:
            self.value.setText(text)
            self.value.setStyleSheet(f"{VALUE_STYLE} color: {hex_code};")
        if sub is not None:
            self.sub.setText(sub or " ")

    def mouseReleaseEvent(self, event: QMouseEvent) -> None:
        if self.cid is not None and event.button() == Qt.MouseButton.LeftButton:
            self.ctx.plot(self.cid)
        super().mouseReleaseEvent(event)


class FlagGrid(QWidget):
    """The bits of a status word, one badge each."""

    def __init__(self, bits: list[tuple[int, str, str]], fault_mask: int = 0, columns: int = 4) -> None:
        super().__init__()
        self.bits = bits
        self.fault_mask = fault_mask
        grid = QGridLayout(self)
        grid.setContentsMargins(0, 0, 0, 0)
        grid.setSpacing(6)
        self.badges: list[Badge] = []
        for i, (_bit, name, tip) in enumerate(bits):
            badge = Badge(name, compact=True)
            badge.setToolTip(tip)
            grid.addWidget(badge, i // columns, i % columns)
            self.badges.append(badge)

    def show_value(self, value: int | None, stale: bool = False) -> None:
        for badge, (bit, name, _tip) in zip(self.badges, self.bits):
            mask = 1 << bit
            if value is None:
                hex_code = theme.TEXT_FAINT
            elif stale:
                hex_code = theme.STALE
            elif not value & mask:
                hex_code = theme.TEXT_FAINT
            else:
                hex_code = theme.FAULT if mask & self.fault_mask else theme.OK
            badge.set_state(name, hex_code)


class Segmented(QWidget):
    """A row of exclusive buttons, like a segmented control."""

    changed = Signal(int)

    def __init__(self, options: list[str], current: int = 0) -> None:
        super().__init__()
        row = QHBoxLayout(self)
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(0)
        self.group = QButtonGroup(self)
        self.group.setExclusive(True)
        for i, text in enumerate(options):
            b = QPushButton(text)
            b.setObjectName("segment")
            b.setCheckable(True)
            b.setChecked(i == current)
            b.setCursor(Qt.CursorShape.PointingHandCursor)
            self.group.addButton(b, i)
            row.addWidget(b)
        self.group.idClicked.connect(self.changed)

    def current(self) -> int:
        return self.group.checkedId()


def style_plot(plot: pg.PlotItem) -> None:
    """Colour a plot's axes and grid for the current light or dark mode."""
    for name in ("left", "bottom", "right", "top"):
        axis = plot.getAxis(name)
        axis.setPen(pg.mkPen(theme.BORDER))
        axis.setTextPen(pg.mkPen(theme.TEXT_MUTED))
    plot.showGrid(x=True, y=True, alpha=0.25)
    for text in (plot.titleLabel,):
        text.setAttr("color", theme.TEXT_MUTED)
    if plot.legend is not None:
        plot.legend.setLabelTextColor(theme.TEXT_MUTED)
