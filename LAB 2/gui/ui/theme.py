"""Dark / light palettes and the application stylesheet (QSS)."""
from __future__ import annotations

from string import Template

PALETTES = {
    "dark": dict(
        bg="#12151a", surface="#1a1f26", surface2="#222932", border="#2e3641", text="#e7eaf0",
        muted="#8a95a6", accent="#4c9aff", accent_hover="#6cb0ff", accent_press="#3480e0",
        on_accent="#0b1220", danger="#ef5350", danger_hover="#f77370", danger_press="#d4403d",
        ok="#3fcf9e", warn="#f2b84b", disabled_bg="#1e242c", disabled_fg="#576170",
        plot_bg="#161a20", plot_fg="#aab3c2", grid="#2a323d",
    ),
    "light": dict(
        bg="#f2f4f8", surface="#ffffff", surface2="#eaeef4", border="#d3d9e3", text="#1c2430",
        muted="#647084", accent="#2563eb", accent_hover="#3b78f0", accent_press="#1d4fc4",
        on_accent="#ffffff", danger="#d93636", danger_hover="#e35252", danger_press="#b82b2b",
        ok="#12946a", warn="#b7791f", disabled_bg="#e6eaf0", disabled_fg="#9aa4b4",
        plot_bg="#ffffff", plot_fg="#3d4859", grid="#e2e7ee",
    ),
}

RUN_COLORS = {
    "dark": ["#4c9aff", "#f2b84b", "#3fcf9e", "#ef7d57", "#b48cf2", "#5fd0e8", "#e86fa5", "#a3c65a"],
    "light": ["#2563eb", "#c98a0b", "#12946a", "#d0532a", "#7c4dcf", "#0e93b0", "#c23c80", "#6b8f1f"],
}

_QSS = Template("""
* { font-family: "Segoe UI", "Inter", "Helvetica Neue", Arial, sans-serif; font-size: 13px; }
QMainWindow, QWidget#Root { background: $bg; }
QWidget { color: $text; }
QToolTip { background: $surface2; color: $text; border: 1px solid $border; padding: 4px 6px; }

QFrame#Card { background: $surface; border: 1px solid $border; border-radius: 10px; }
QLabel#CardTitle { color: $muted; font-size: 11px; font-weight: 700; letter-spacing: 1px; }
QLabel#Muted { color: $muted; }
QLabel#Big { font-size: 20px; font-weight: 600; }
QLabel { background: transparent; }
QScrollArea, QScrollArea > QWidget > QWidget { background: transparent; border: none; }

QPushButton { background: $surface2; color: $text; border: 1px solid $border; border-radius: 7px;
              padding: 7px 14px; font-weight: 600; }
QPushButton:hover { border-color: $accent; background: $surface; }
QPushButton:pressed { background: $border; }
QPushButton:disabled { background: $disabled_bg; color: $disabled_fg; border-color: $disabled_bg; }
QPushButton:focus { border-color: $accent; }
QPushButton[variant="primary"] { background: $accent; color: $on_accent; border-color: $accent; }
QPushButton[variant="primary"]:hover { background: $accent_hover; border-color: $accent_hover; }
QPushButton[variant="primary"]:pressed { background: $accent_press; border-color: $accent_press; }
QPushButton[variant="primary"]:disabled { background: $disabled_bg; color: $disabled_fg; border-color: $disabled_bg; }
QPushButton[variant="danger"] { background: $danger; color: #ffffff; border-color: $danger; }
QPushButton[variant="danger"]:hover { background: $danger_hover; border-color: $danger_hover; }
QPushButton[variant="danger"]:pressed { background: $danger_press; border-color: $danger_press; }
QPushButton[variant="danger"]:disabled { background: $disabled_bg; color: $disabled_fg; border-color: $disabled_bg; }
QPushButton[variant="ghost"] { background: transparent; border-color: transparent; color: $muted; padding: 5px 9px; }
QPushButton[variant="ghost"]:hover { color: $text; background: $surface2; border-color: $border; }
QPushButton[variant="ghost"]:pressed { background: $border; }
QPushButton[variant="ghost"]:disabled { color: $disabled_fg; background: transparent; }

QComboBox, QLineEdit { background: $surface2; border: 1px solid $border; border-radius: 7px; padding: 6px 9px;
                       selection-background-color: $accent; selection-color: $on_accent; }
QComboBox:hover, QLineEdit:hover { border-color: $muted; }
QComboBox:focus, QLineEdit:focus { border-color: $accent; }
QComboBox:disabled, QLineEdit:disabled { background: $disabled_bg; color: $disabled_fg; }
QComboBox::drop-down { border: none; width: 22px; }
QComboBox QAbstractItemView { background: $surface; border: 1px solid $border; selection-background-color: $accent;
                              selection-color: $on_accent; outline: none; }

QCheckBox { spacing: 8px; }
QCheckBox::indicator { width: 16px; height: 16px; border-radius: 4px; border: 1px solid $muted; background: $surface2; }
QCheckBox::indicator:hover { border-color: $accent; }
QCheckBox::indicator:checked { background: $accent; border-color: $accent; }
QCheckBox::indicator:disabled { background: $disabled_bg; border-color: $disabled_bg; }

QProgressBar { background: $surface2; border: 1px solid $border; border-radius: 7px; height: 14px; text-align: center;
               color: $text; font-size: 11px; }
QProgressBar::chunk { background: $accent; border-radius: 6px; }
QProgressBar[state="receiving"]::chunk { background: $warn; }
QProgressBar[state="idle"]::chunk { background: $ok; }

QTableWidget { background: $surface; alternate-background-color: $surface2; border: 1px solid $border; border-radius: 8px;
               gridline-color: transparent; selection-background-color: $accent; selection-color: $on_accent; outline: none; }
QTableWidget::item { padding: 4px 6px; }
QHeaderView::section { background: $surface2; color: $muted; border: none; border-bottom: 1px solid $border;
                       padding: 6px; font-weight: 700; font-size: 11px; }
QPlainTextEdit { background: $plot_bg; border: 1px solid $border; border-radius: 8px; color: $plot_fg;
                 font-family: "Cascadia Mono", "Consolas", monospace; font-size: 12px; }

QScrollBar:vertical { background: transparent; width: 10px; margin: 2px; }
QScrollBar::handle:vertical { background: $border; border-radius: 4px; min-height: 24px; }
QScrollBar::handle:vertical:hover { background: $muted; }
QScrollBar:horizontal { background: transparent; height: 10px; margin: 2px; }
QScrollBar::handle:horizontal { background: $border; border-radius: 4px; min-width: 24px; }
QScrollBar::add-line, QScrollBar::sub-line { width: 0; height: 0; }
QSplitter::handle { background: transparent; }

QLabel[badge] { border-radius: 9px; padding: 2px 10px; font-size: 11px; font-weight: 700; }
QLabel[badge="pass"] { background: $ok; color: #06281d; }
QLabel[badge="warn"] { background: $warn; color: #2e2005; }
QLabel[badge="fail"] { background: $danger; color: #ffffff; }
QLabel[badge="none"] { background: $surface2; color: $muted; }

QLabel#Dot { border-radius: 6px; min-width: 12px; max-width: 12px; min-height: 12px; max-height: 12px; }
QLabel#Dot[conn="off"] { background: $muted; }
QLabel#Dot[conn="on"] { background: $ok; }
QLabel#Dot[conn="busy"] { background: $warn; }

QFrame#Banner { border-radius: 8px; border: 1px solid $danger; background: $surface; }
QFrame#Banner[level="info"] { border-color: $accent; }
QFrame#Banner[level="warn"] { border-color: $warn; }
QStatusBar { background: $surface; color: $muted; border-top: 1px solid $border; }
""")


def build_qss(theme: str) -> str:
    return _QSS.substitute(PALETTES[theme])
