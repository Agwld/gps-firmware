"""Colours, fonts and the application stylesheet.

Colour means state. Everything else stays neutral (white in light mode, dark grey in dark mode),
with SUFST blue kept for selection and branding. Screens take colours from here and never define their own.
"""

from collections.abc import Callable
from pathlib import Path

from PySide6.QtCore import QObject, QSettings, Signal
from PySide6.QtGui import QColor, QFont, QPalette
from PySide6.QtWidgets import QApplication

# Two palettes with the same names. set_mode() copies one into the module globals below, so
# code reads theme.TEXT and so on at the moment it draws and always gets the current mode.
LIGHT = {
    # Surfaces and text
    "BG": "#ffffff",
    "SURFACE": "#f5f7fa",  # Rail, side panel, cards
    "SURFACE_2": "#ffffff",  # Inputs and buttons
    "SURFACE_3": "#e8eef6",  # Hover and selection
    "BORDER": "#d9dfe7",
    "TEXT": "#14202e",
    "TEXT_MUTED": "#5b6676",
    "TEXT_FAINT": "#8d96a3",
    "TOOLTIP_BG": "#ffffff",
    # SUFST blue. Selection and branding only, never data state.
    "ACCENT": "#00366a",
    "ACCENT_HOVER": "#1b5c9b",
    "ACCENT_TINT": "#e6eef7",  # Selected backgrounds
    "ON_ACCENT": "#ffffff",  # Text on an accent background
    "DANGER_BORDER": "#f0b8bb",
    "DANGER_HOVER": "#fdecee",
    # Data state, dark enough to read on white
    "NEUTRAL": "#9aa5b4",  # A healthy value that needs no attention.
    "OK": "#1a7f37",
    "WARN": "#c88a12",
    "FAULT": "#d1242f",
    "STALE": "#c3c9d2",
    # Plot series, where colour tells lines apart rather than showing state.
    "SERIES": ["#2f6fdb", "#e8710a", "#12a090", "#c2399b", "#8a6d00", "#6f42c1", "#0b8bb8", "#d6453d"],
}

DARK = {
    "BG": "#0e1014",
    "SURFACE": "#15181e",
    "SURFACE_2": "#1c2028",
    "SURFACE_3": "#252a34",
    "BORDER": "#2a2f3a",
    "TEXT": "#e6e8ee",
    "TEXT_MUTED": "#9098a8",
    "TEXT_FAINT": "#5d6474",
    "TOOLTIP_BG": "#252a34",
    # SUFST navy is too dark to see on black, so dark mode uses the lighter blue from the logo.
    "ACCENT": "#3a86d4",
    "ACCENT_HOVER": "#5a9be0",
    "ACCENT_TINT": "#14263b",
    "ON_ACCENT": "#ffffff",
    "DANGER_BORDER": "#5a2a2a",
    "DANGER_HOVER": "#2a1a1c",
    "NEUTRAL": "#8b95a7",
    "OK": "#3fb950",
    "WARN": "#e3b341",
    "FAULT": "#f85149",
    "STALE": "#4a505c",
    "SERIES": ["#5fa8ff", "#f0883e", "#56d4a8", "#e86fbf", "#d4c35a", "#9d8cff", "#6fd0e8", "#ff8f8f"],
}

PALETTES = {"light": LIGHT, "dark": DARK}
mode = "light"

BG = SURFACE = SURFACE_2 = SURFACE_3 = BORDER = TEXT = TEXT_MUTED = TEXT_FAINT = TOOLTIP_BG = ""
ACCENT = ACCENT_HOVER = ACCENT_TINT = ON_ACCENT = DANGER_BORDER = DANGER_HOVER = ""
NEUTRAL = OK = WARN = FAULT = STALE = ""
SERIES: list[str] = []
globals().update(LIGHT)


class _Signals(QObject):
    changed = Signal()  # The mode switched; anything holding colours should redraw.


signals = _Signals()
_hooks: list[Callable[[], None]] = []  # Run before `changed`, for modules caching colours


def on_change(hook: Callable[[], None]) -> None:
    _hooks.append(hook)


MONO_FAMILIES = ["Cascadia Mono", "Consolas", "Menlo", "DejaVu Sans Mono", "monospace"]


def color(hex_code: str, alpha: int = 255) -> QColor:
    c = QColor(hex_code)
    c.setAlpha(alpha)
    return c


def mix(a: str | QColor, b: str | QColor, f: float) -> QColor:
    """Blend from a (f=0) to b (f=1)."""
    ca, cb = QColor(a), QColor(b)
    f = max(0.0, min(1.0, f))
    return QColor(
        round(ca.red() + (cb.red() - ca.red()) * f),
        round(ca.green() + (cb.green() - ca.green()) * f),
        round(ca.blue() + (cb.blue() - ca.blue()) * f),
    )


def mono_font(size: int = 12) -> QFont:
    font = QFont()
    font.setFamilies(MONO_FAMILIES)
    font.setStyleHint(QFont.StyleHint.Monospace)
    font.setPointSize(size)
    return font


def saved_mode() -> str:
    value = QSettings("SUFST", "GPSDesktop").value("theme", "light")
    return value if value in PALETTES else "light"


def set_mode(new_mode: str, app: QApplication | None = None) -> None:
    """Switch between "light" and "dark", remember the choice, and tell everything to redraw."""
    global mode
    mode = new_mode if new_mode in PALETTES else "light"
    globals().update(PALETTES[mode])
    QSettings("SUFST", "GPSDesktop").setValue("theme", mode)
    app = app or QApplication.instance()
    if app is not None:
        _apply_style(app)
    for hook in _hooks:
        hook()
    signals.changed.emit()


def toggle() -> None:
    set_mode("dark" if mode == "light" else "light")


def apply(app: QApplication) -> None:
    """Style the app in the mode chosen last time."""
    global mode
    mode = saved_mode()
    globals().update(PALETTES[mode])
    _apply_style(app)
    for hook in _hooks:
        hook()


def _apply_style(app: QApplication) -> None:
    app.setStyle("Fusion")
    palette = QPalette()
    roles = {
        QPalette.ColorRole.Window: BG,
        QPalette.ColorRole.WindowText: TEXT,
        QPalette.ColorRole.Base: SURFACE_2,
        QPalette.ColorRole.AlternateBase: SURFACE,
        QPalette.ColorRole.Text: TEXT,
        QPalette.ColorRole.Button: SURFACE_2,
        QPalette.ColorRole.ButtonText: TEXT,
        QPalette.ColorRole.Highlight: ACCENT,
        QPalette.ColorRole.HighlightedText: ON_ACCENT,
        QPalette.ColorRole.ToolTipBase: SURFACE_3,
        QPalette.ColorRole.ToolTipText: TEXT,
        QPalette.ColorRole.PlaceholderText: TEXT_FAINT,
        QPalette.ColorRole.Light: SURFACE_3,
        QPalette.ColorRole.Midlight: SURFACE_3,
        QPalette.ColorRole.Mid: BORDER,
        QPalette.ColorRole.Dark: BORDER,
        QPalette.ColorRole.Shadow: TEXT_FAINT,
        QPalette.ColorRole.Link: ACCENT,
    }
    for role, value in roles.items():
        palette.setColor(role, QColor(value))
    for role in (QPalette.ColorRole.Text, QPalette.ColorRole.WindowText, QPalette.ColorRole.ButtonText):
        palette.setColor(QPalette.ColorGroup.Disabled, role, QColor(TEXT_FAINT))
    app.setPalette(palette)
    app.setStyleSheet(stylesheet() + _spin_buttons())


def stylesheet() -> str:
    return f"""
QToolTip {{
    background: {TOOLTIP_BG}; color: {TEXT}; border: 1px solid {BORDER}; padding: 6px;
}}

/* Navigation rail */
#navRail {{ background: {SURFACE}; border-right: 1px solid {BORDER}; }}
#navRail QPushButton#navButton {{
    background: transparent; border: none; border-left: 3px solid transparent;
    color: {TEXT_MUTED}; text-align: left; padding: 9px 14px 9px 13px; font-size: 13px;
}}
#navRail QPushButton#navButton:hover {{ color: {TEXT}; background: {SURFACE_3}; }}
#navRail QPushButton#navButton:checked {{
    color: {ON_ACCENT}; background: {ACCENT}; border-left: 3px solid {ACCENT}; font-weight: 600;
}}
#navRail QPushButton#navButton:checked:hover {{ background: {ACCENT_HOVER}; border-left-color: {ACCENT_HOVER}; }}
#logoToggle {{ border: none; border-radius: 6px; padding: 3px; background: transparent; }}
#logoToggle:hover {{ background: {SURFACE_3}; }}
#logoToggle:pressed {{ background: {BORDER}; }}
#brand {{ color: {ACCENT}; font-size: 15px; font-weight: 600; }}

/* Per-screen configuration panel */
#sidePanel {{ background: {SURFACE}; border-right: 1px solid {BORDER}; }}
#sidePanel QScrollArea, #sidePanel QScrollArea > QWidget > QWidget {{ background: transparent; border: none; }}
#panelTitle {{ font-size: 15px; font-weight: 600; color: {TEXT}; }}
#sectionLabel {{
    color: {TEXT_FAINT}; font-size: 11px; font-weight: 600; letter-spacing: 1px;
    padding-top: 10px;
}}
#muted {{ color: {TEXT_MUTED}; }}
#faint {{ color: {TEXT_FAINT}; }}

/* Content */
#screenTitle {{ font-size: 18px; font-weight: 600; color: {TEXT}; }}
#screenSummary {{ color: {TEXT_MUTED}; }}
QFrame#card {{ background: {SURFACE}; border: 1px solid {BORDER}; border-radius: 8px; }}
#cardTitle {{ color: {TEXT_MUTED}; font-size: 12px; font-weight: 600; }}
#tileCaption {{ color: {TEXT_MUTED}; font-size: 12px; }}
#tileSub {{ color: {TEXT_FAINT}; font-size: 11px; }}

/* Controls */
QPushButton {{
    background: {SURFACE_2}; color: {TEXT}; border: 1px solid {BORDER}; border-radius: 6px;
    padding: 6px 12px;
}}
QPushButton:hover {{ background: {SURFACE_3}; }}
QPushButton:pressed {{ background: {BORDER}; }}
QPushButton:disabled {{ color: {TEXT_FAINT}; background: {SURFACE}; }}
QPushButton:checked {{ background: {ACCENT_TINT}; border-color: {ACCENT}; color: {ACCENT}; }}
QPushButton#primary {{ background: {ACCENT}; border-color: {ACCENT}; color: {ON_ACCENT}; font-weight: 600; }}
QPushButton#primary:hover {{ background: {ACCENT_HOVER}; border-color: {ACCENT_HOVER}; }}
QPushButton#primary:disabled {{ background: {SURFACE_2}; border-color: {BORDER}; color: {TEXT_FAINT}; }}
QPushButton#danger {{ color: {FAULT}; border-color: {DANGER_BORDER}; }}
QPushButton#danger:hover {{ background: {DANGER_HOVER}; }}
QPushButton#segment {{ border-radius: 0; padding: 5px 10px; }}

QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox {{
    background: {SURFACE_2}; border: 1px solid {BORDER}; border-radius: 5px; padding: 4px 6px;
    color: {TEXT}; selection-background-color: {ACCENT};
}}
QLineEdit:focus, QSpinBox:focus, QDoubleSpinBox:focus, QComboBox:focus {{ border-color: {ACCENT}; }}
QComboBox QAbstractItemView {{ background: {SURFACE_2}; border: 1px solid {BORDER}; selection-background-color: {ACCENT_TINT}; selection-color: {ACCENT}; }}
QCheckBox {{ spacing: 8px; }}

QTableWidget, QListWidget, QPlainTextEdit, QTextEdit {{
    background: {SURFACE}; border: 1px solid {BORDER}; border-radius: 6px; gridline-color: {BORDER};
}}
QTableWidget::item, QListWidget::item {{ padding: 4px; }}
QTableWidget::item:selected, QListWidget::item:selected {{ background: {ACCENT_TINT}; color: {TEXT}; }}
QHeaderView::section {{
    background: {SURFACE}; color: {TEXT_MUTED}; border: none; border-bottom: 1px solid {BORDER};
    padding: 6px; font-weight: 600;
}}
QTableCornerButton::section {{ background: {SURFACE}; border: none; }}

QTabWidget::pane {{ border: none; }}
QTabBar::tab {{
    background: transparent; color: {TEXT_MUTED}; padding: 7px 14px; border: none;
    border-bottom: 2px solid transparent;
}}
QTabBar::tab:selected {{ color: {ACCENT}; border-bottom: 2px solid {ACCENT}; }}
QTabBar::tab:hover {{ color: {TEXT}; }}

QSplitter::handle {{ background: {BG}; }}
QProgressBar {{ background: {BORDER}; border: none; border-radius: 4px; height: 8px; text-align: center; }}
QProgressBar::chunk {{ border-radius: 4px; background: {NEUTRAL}; }}

QScrollBar:vertical {{ background: transparent; width: 10px; margin: 0; }}
QScrollBar:horizontal {{ background: transparent; height: 10px; margin: 0; }}
QScrollBar::handle {{ background: {BORDER}; border-radius: 4px; min-height: 24px; min-width: 24px; }}
QScrollBar::handle:hover {{ background: {TEXT_FAINT}; }}
QScrollBar::add-line, QScrollBar::sub-line {{ width: 0; height: 0; }}
QScrollBar::add-page, QScrollBar::sub-page {{ background: transparent; }}

QStatusBar {{ background: {SURFACE}; color: {TEXT_MUTED}; border-top: 1px solid {BORDER}; }}
QMenuBar {{ background: {SURFACE}; color: {TEXT_MUTED}; }}
QMenuBar::item:selected {{ background: {SURFACE_3}; color: {TEXT}; }}
QMenu {{ background: {SURFACE_2}; border: 1px solid {BORDER}; }}
QMenu::item:selected {{ background: {SURFACE_3}; }}
"""


def _spin_buttons() -> str:
    """Spin box arrows. Styling a spin box's frame hides Fusion's own arrows, so draw them here."""
    assets = (Path(__file__).resolve().parent / "assets").as_posix()
    return f"""
QAbstractSpinBox::up-button, QAbstractSpinBox::down-button {{
    subcontrol-origin: border; width: 18px; border: none; border-left: 1px solid {BORDER}; background: {SURFACE_3};
}}
QAbstractSpinBox::up-button {{ subcontrol-position: top right; border-top-right-radius: 5px; }}
QAbstractSpinBox::down-button {{ subcontrol-position: bottom right; border-bottom-right-radius: 5px; }}
QAbstractSpinBox::up-button:hover, QAbstractSpinBox::down-button:hover {{ background: {BORDER}; }}
QAbstractSpinBox::up-arrow {{ image: url({assets}/arrow_up.svg); width: 8px; height: 5px; }}
QAbstractSpinBox::down-arrow {{ image: url({assets}/arrow_down.svg); width: 8px; height: 5px; }}
QComboBox::drop-down {{ border: none; width: 20px; }}
QComboBox::down-arrow {{ image: url({assets}/arrow_down.svg); width: 8px; height: 5px; }}
"""
