from __future__ import annotations

QSS = """
* { font-family: "Segoe UI", "Inter", Arial, sans-serif; font-size: 13px; }
QMainWindow, QWidget { background: #12151a; color: #e7eaf0; }
QGroupBox { border: 1px solid #2e3641; border-radius: 8px; margin-top: 10px; padding-top: 10px; font-weight: 600; }
QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 4px; color: #8a95a6; }
QPushButton { background: #222932; border: 1px solid #2e3641; border-radius: 7px; padding: 7px 14px; font-weight: 600; }
QPushButton:hover { border-color: #4c9aff; }
QPushButton:pressed { background: #2e3641; }
QPushButton[variant="primary"] { background: #4c9aff; color: #0b1220; border-color: #4c9aff; }
QPushButton[variant="danger"] { background: #ef5350; color: #ffffff; border-color: #ef5350; }
QComboBox, QLineEdit { background: #1a1f26; border: 1px solid #2e3641; border-radius: 6px; padding: 5px 8px; }
QSlider::groove:horizontal { height: 6px; background: #222932; border-radius: 3px; }
QSlider::handle:horizontal { width: 16px; background: #4c9aff; border-radius: 8px; margin: -6px 0; }
QLabel#Value { color: #3fcf9e; font-weight: 700; }
QLabel#Status { color: #8a95a6; }
QPlainTextEdit { background: #161a20; border: 1px solid #2e3641; border-radius: 6px;
                 color: #aab3c2; font-family: "Cascadia Mono", "Consolas", monospace; font-size: 12px; }
"""

PLOT_BG = "#161a20"
PLOT_FG = "#aab3c2"
GRID = "#2a323d"
REF_COLOR = "#3fcf9e"
MEAS_COLOR = "#4c9aff"
PWM_COLOR = "#e7eaf0"
ERR_COLOR = "#f2b84b"
