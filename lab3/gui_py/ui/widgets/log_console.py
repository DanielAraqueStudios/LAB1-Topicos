from __future__ import annotations

import html
import time

from PyQt6.QtGui import QTextCursor
from PyQt6.QtWidgets import QHBoxLayout, QPlainTextEdit, QPushButton, QVBoxLayout, QWidget

_PREFIX = {"tx": ">>", "rx": "<<", "sys": "--"}
_COLOR = {"tx": "#4c9aff", "rx": "#3fcf9e", "sys": "#f2b84b"}


class LogConsole(QWidget):
    def __init__(self) -> None:
        super().__init__()
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(6)
        bar = QHBoxLayout()
        self.title_lbl = QPushButton("Log serial")
        self.title_lbl.setFlat(True)
        self.title_lbl.setEnabled(False)
        self.clear_btn = QPushButton("Limpiar")
        self.clear_btn.setProperty("variant", "ghost")
        bar.addWidget(self.title_lbl)
        bar.addStretch(1)
        bar.addWidget(self.clear_btn)
        lay.addLayout(bar)
        self.text = QPlainTextEdit()
        self.text.setReadOnly(True)
        self.text.setMaximumBlockCount(500)
        lay.addWidget(self.text)
        self.clear_btn.clicked.connect(self.text.clear)

    def append_batch(self, items: list) -> None:
        stamp = time.strftime("%H:%M:%S")
        for d, t in items:
            color = _COLOR.get(d, "#aab3c2")
            prefix = _PREFIX.get(d, "--")
            line = html.escape(f"{stamp} {prefix} {t}")
            self.text.appendHtml(f'<span style="color:{color}">{line}</span>')
        self.text.moveCursor(QTextCursor.MoveOperation.End)
        self.text.ensureCursorVisible()
