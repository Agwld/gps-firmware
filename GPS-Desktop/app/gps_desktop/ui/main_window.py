"""The main window: start screen, then navigation rail, side panel and the screens."""

from __future__ import annotations

from pathlib import Path

from PySide6.QtCore import QSize, Qt, QTimer
from PySide6.QtGui import QCloseEvent, QIcon, QKeySequence, QShortcut
from PySide6.QtWidgets import (
    QFrame,
    QHBoxLayout,
    QMainWindow,
    QPushButton,
    QScrollArea,
    QStackedWidget,
    QToolButton,
    QVBoxLayout,
    QWidget,
)

from gps_desktop import theme
from gps_desktop.ui.connection import ConnectionPage
from gps_desktop.ui.context import AppContext
from gps_desktop.ui.screens.dashboard import DashboardScreen
from gps_desktop.ui.screens.laps import LapsScreen
from gps_desktop.ui.screens.plots import PlotsScreen
from gps_desktop.ui.screens.signals import SignalsScreen
from gps_desktop.ui.screens.system import SystemScreen
from gps_desktop.ui.screens.track import TrackScreen
from gps_desktop.ui.widgets import button, label

SCREENS = [DashboardScreen, TrackScreen, LapsScreen, PlotsScreen, SignalsScreen, SystemScreen]


class NavButton(QPushButton):
    def __init__(self, text: str) -> None:
        super().__init__(text)
        self.setObjectName("navButton")
        self.setCheckable(True)
        self.setCursor(Qt.CursorShape.PointingHandCursor)


class Banner(QFrame):
    """Shown above every screen when data is stale or nothing is connected."""

    def __init__(self, ctx: AppContext) -> None:
        super().__init__()
        self.ctx = ctx
        row = QHBoxLayout(self)
        row.setContentsMargins(16, 8, 12, 8)
        self.text = label("")
        self.text.setTextFormat(Qt.TextFormat.RichText)
        row.addWidget(self.text, 1)
        self.action = button("Connect…")
        row.addWidget(self.action)
        self.timer = QTimer(self)
        self.timer.setInterval(500)
        self.timer.timeout.connect(self.refresh)
        self.timer.start()
        ctx.data.staleChanged.connect(self.refresh)
        ctx.linkChanged.connect(self.refresh)

    def _style(self, hex_code: str) -> None:
        c = theme.color(hex_code)
        self.setStyleSheet(f"Banner {{ background: rgba({c.red()},{c.green()},{c.blue()},30); "
                           f"border-bottom: 1px solid rgba({c.red()},{c.green()},{c.blue()},90); }}")

    def refresh(self) -> None:
        data = self.ctx.data
        muted = theme.TEXT_MUTED
        if not self.ctx.connected:
            self._style(theme.NEUTRAL)
            kept = " Values shown are the last ones received." if data.values else ""
            self.text.setText(f"<b>Offline</b> <span style='color:{muted}'>· Not connected to a GPS node.{kept}</span>")
            self.action.show()
            self.show()
            return
        self.action.hide()
        stale = sorted(data.stale)
        if not stale:
            self.hide()
            return
        if len(stale) == sum(1 for name in data.messages if data.stale_after(name) is not None):
            what = "No data from the GPS node"
        else:
            names = ", ".join(stale[:4]) + (f" and {len(stale) - 4} more" if len(stale) > 4 else "")
            what = f"Not arriving: {names}"
        self._style(theme.WARN)
        self.text.setText(f"<b style='color:{theme.WARN}'>Stale data</b> <span style='color:{muted}'>· {what}. "
                          "Affected values are greyed out.</span>")
        self.show()


class MainWindow(QMainWindow):
    def __init__(self, ctx: AppContext | None = None) -> None:
        super().__init__()
        self.ctx = ctx or AppContext(self)
        self.setWindowTitle("GPS Desktop")
        self.resize(1440, 900)
        self.setMinimumSize(1100, 700)

        self.pages = QStackedWidget()
        self.setCentralWidget(self.pages)

        self.connection = ConnectionPage()
        self.connection.connectSimulator.connect(lambda: self._connect(self.ctx.connect_simulator))
        self.connection.connectLive.connect(
            lambda interface, channel, bitrate: self._connect(lambda: self.ctx.connect_live(interface, channel, bitrate)))
        self.connection.connectReplay.connect(
            lambda path, speed, loop: self._connect(lambda: self.ctx.connect_replay(path, speed, loop)))
        self.pages.addWidget(self.connection)

        self.workspace = QWidget()
        self.pages.addWidget(self.workspace)
        row = QHBoxLayout(self.workspace)
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(0)

        self.screens = [cls(self.ctx) for cls in SCREENS]

        # Navigation rail
        rail = QFrame()
        rail.setObjectName("navRail")
        rail.setFixedWidth(178)
        rail_box = QVBoxLayout(rail)
        rail_box.setContentsMargins(0, 14, 0, 12)
        rail_box.setSpacing(2)
        brand = QHBoxLayout()
        brand.setContentsMargins(14, 0, 14, 12)
        # The logo doubles as the light/dark mode switch.
        self.logo = QToolButton()
        self.logo.setObjectName("logoToggle")
        self.logo.setIcon(QIcon(str(Path(__file__).resolve().parent.parent / "assets" / "icon.png")))
        self.logo.setIconSize(QSize(26, 26))
        self.logo.setCursor(Qt.CursorShape.PointingHandCursor)
        self.logo.setAutoRaise(True)
        self.logo.clicked.connect(theme.toggle)
        brand.addWidget(self.logo)
        brand.addWidget(label("GPS Desktop", "brand"), 1)
        rail_box.addLayout(brand)
        self.nav: list[NavButton] = []
        for i, screen in enumerate(self.screens):
            b = NavButton(screen.nav_label)
            b.setToolTip(f"{screen.title} (Ctrl+{i + 1})")
            b.clicked.connect(lambda _checked=False, index=i: self.show_screen(index))
            rail_box.addWidget(b)
            self.nav.append(b)
            shortcut = QShortcut(QKeySequence(f"Ctrl+{i + 1}"), self)
            shortcut.activated.connect(lambda index=i: self.show_screen(index))
        rail_box.addStretch(1)
        self.link_label = label("", "muted", wrap=True)
        self.link_label.setTextFormat(Qt.TextFormat.RichText)
        self.link_label.setContentsMargins(14, 0, 14, 4)
        rail_box.addWidget(self.link_label)
        link_row = QHBoxLayout()
        link_row.setContentsMargins(12, 0, 12, 0)
        self.link_button = button("Disconnect", self._toggle_link)
        link_row.addWidget(self.link_button)
        rail_box.addLayout(link_row)
        row.addWidget(rail)

        # Side panel for the current screen
        side = QFrame()
        side.setObjectName("sidePanel")
        side.setFixedWidth(300)
        side_box = QVBoxLayout(side)
        side_box.setContentsMargins(0, 0, 0, 0)
        self.panels = QStackedWidget()
        for screen in self.screens:
            scroll = QScrollArea()
            scroll.setWidgetResizable(True)
            scroll.setFrameShape(QScrollArea.Shape.NoFrame)
            scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
            scroll.setWidget(screen.panel)
            self.panels.addWidget(scroll)
        side_box.addWidget(self.panels)
        row.addWidget(side)

        # Content
        content = QWidget()
        content_box = QVBoxLayout(content)
        content_box.setContentsMargins(0, 0, 0, 0)
        content_box.setSpacing(0)
        self.banner = Banner(self.ctx)
        self.banner.action.clicked.connect(self._go_to_connection)
        content_box.addWidget(self.banner)
        self.stack = QStackedWidget()
        for screen in self.screens:
            self.stack.addWidget(screen)
        content_box.addWidget(self.stack, 1)
        row.addWidget(content, 1)

        self.statusBar().setSizeGripEnabled(False)
        self.ctx.message.connect(lambda text: self.statusBar().showMessage(text, 4000))
        self.ctx.linkChanged.connect(self._link_changed)
        self.ctx.linkFailed.connect(self._link_failed)
        self.ctx.recordingChanged.connect(self._link_changed)

        theme.signals.changed.connect(self._restyle)
        self._restyle()
        self.show_screen(0)
        self.pages.setCurrentWidget(self.connection)
        self.statusBar().hide()

    def show_screen(self, index: int) -> None:
        if self.pages.currentWidget() is not self.workspace:
            return
        for i, b in enumerate(self.nav):
            b.setChecked(i == index)
        self.stack.setCurrentIndex(index)
        self.panels.setCurrentIndex(index)

    def _connect(self, start) -> None:
        start()
        self.pages.setCurrentWidget(self.workspace)
        self.statusBar().show()
        self.show_screen(0)

    def _go_to_connection(self) -> None:
        self.statusBar().hide()
        self.pages.setCurrentWidget(self.connection)

    def _toggle_link(self) -> None:
        if self.ctx.connected:
            self.ctx.disconnect_link()
        self._go_to_connection()

    def _link_failed(self, error: str) -> None:
        """An adapter that would not open, or a log that could not be read: back to the start screen."""
        self._go_to_connection()
        self.connection.show_error(f"Could not connect: {error}")

    def _link_changed(self) -> None:
        if self.ctx.connected:
            recording = f"<br><span style='color:{theme.FAULT}'>●</span> Recording" if self.ctx.recording else ""
            self.link_label.setText(f"<span style='color:{theme.OK}'>●</span> {self.ctx.data.link_name}{recording}")
            self.link_button.setText("Disconnect")
        else:
            self.link_label.setText(f"<span style='color:{theme.TEXT_FAINT}'>●</span> Offline")
            self.link_button.setText("Connect…")

    def _restyle(self) -> None:
        other = "light" if theme.mode == "dark" else "dark"
        self.logo.setToolTip(f"Switch to {other} mode")
        self.banner.refresh()
        self._link_changed()

    def closeEvent(self, event: QCloseEvent) -> None:
        self.ctx.disconnect_link()
        super().closeEvent(event)
