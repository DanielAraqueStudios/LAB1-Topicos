"""Collapsible raw serial log."""
from __future__ import annotations

import time

from PyQt6.QtWidgets import QHBoxLayout, QPlainTextEdit, QVBoxLayout, QWidget

from .common import make_button

_PREFIX = {"tx": ">>", "rx": "<<", "sys": "--"}


class LogConsole(QWidget):
    def __init__(self) -> None:
        super().__init__()
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(6)
        bar = QHBoxLayout()
        self.toggle_btn = make_button("Serial log", "ghost", "Show or hide the raw serial log (Ctrl+L)")
        self.clear_btn = make_button("Clear", "ghost", "Clear the log")
        bar.addWidget(self.toggle_btn)
        bar.addStretch(1)
        bar.addWidget(self.clear_btn)
        lay.addLayout(bar)
        self.text = QPlainTextEdit()
        self.text.setReadOnly(True)
        self.text.setMaximumBlockCount(4000)
        self.text.setMinimumHeight(110)
        self.text.setMaximumHeight(220)
        lay.addWidget(self.text)
        self.toggle_btn.clicked.connect(self.toggle)
        self.clear_btn.clicked.connect(self.text.clear)
        self.set_expanded(False)

    def set_expanded(self, on: bool) -> None:
        self._expanded = on
        self.text.setVisible(on)
        self.clear_btn.setVisible(on)
        self.toggle_btn.setText(("v " if on else "> ") + "Serial log")

    def toggle(self) -> None:
        self.set_expanded(not self._expanded)

    def append_batch(self, items: list) -> None:
        stamp = time.strftime("%H:%M:%S")
        self.text.appendPlainText("\n".join(f"{stamp} {_PREFIX.get(d, '--')} {t}" for d, t in items))
