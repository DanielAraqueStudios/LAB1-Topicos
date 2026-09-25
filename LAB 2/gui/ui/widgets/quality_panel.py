"""Data-quality summary for the selected run."""
from __future__ import annotations

from PyQt6.QtWidgets import QGridLayout, QHBoxLayout, QLabel, QVBoxLayout, QWidget

from core.quality import QualityReport
from .common import make_badge, make_card, set_prop

_TEXT = {"pass": "PASS", "warn": "WARN", "fail": "FAIL", "none": "-"}


class QualityPanel(QWidget):
    def __init__(self) -> None:
        super().__init__()
        card, lay = make_card("Data quality")
        outer = QHBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.addWidget(card)

        head = QHBoxLayout()
        self.title = QLabel("No run selected")
        self.title.setWordWrap(True)
        self.title.setObjectName("Big")
        self.overall = make_badge("-", "none")
        head.addWidget(self.title, 1)
        head.addWidget(self.overall)
        lay.addLayout(head)

        self.grid_host = QWidget()
        self.grid = QGridLayout(self.grid_host)
        self.grid.setContentsMargins(0, 0, 0, 0)
        self.grid.setHorizontalSpacing(10)
        self.grid.setVerticalSpacing(8)
        self.grid.setColumnStretch(2, 2)
        self.grid.setColumnStretch(1, 1)
        lay.addWidget(self.grid_host)
        lay.addStretch(1)

    def _clear(self) -> None:
        while self.grid.count():
            w = self.grid.takeAt(0).widget()
            if w:
                w.deleteLater()

    def clear(self) -> None:
        self._clear()
        self.title.setText("No run selected")
        self.overall.setText("-")
        set_prop(self.overall, "badge", "none")

    def show_report(self, name: str, rep: QualityReport) -> None:
        self._clear()
        self.title.setText(name)
        self.overall.setText(_TEXT[rep.overall])
        set_prop(self.overall, "badge", rep.overall)
        for row, c in enumerate(rep.checks):
            badge = make_badge(_TEXT[c.status], c.status)
            label = QLabel(c.label)
            label.setWordWrap(True)
            label.setMinimumWidth(90)
            detail = QLabel(c.detail)
            detail.setObjectName("Muted")
            detail.setWordWrap(True)
            detail.setToolTip(c.detail)
            self.grid.addWidget(badge, row, 0)
            self.grid.addWidget(label, row, 1)
            self.grid.addWidget(detail, row, 2)
