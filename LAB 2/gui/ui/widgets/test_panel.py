"""Step / PRBS / Abort controls with progress."""
from __future__ import annotations

from PyQt6.QtCore import pyqtSignal
from PyQt6.QtWidgets import QHBoxLayout, QLabel, QProgressBar, QWidget

from .common import make_button, make_card, set_prop


class TestPanel(QWidget):
    __test__ = False  # not a pytest class
    start_requested = pyqtSignal(str)
    abort_requested = pyqtSignal()

    def __init__(self) -> None:
        super().__init__()
        card, lay = make_card("Test")
        outer = QHBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.addWidget(card)

        row = QHBoxLayout()
        self.step_btn = make_button("Step", "primary", "Run a step test (Ctrl+1)")
        self.prbs_btn = make_button("PRBS", "primary", "Run a PRBS test (Ctrl+2)")
        self.abort_btn = make_button("Abort", "danger", "Stop the test and force the output to 0 (Esc)")
        for b in (self.step_btn, self.prbs_btn, self.abort_btn):
            row.addWidget(b, 1)
        lay.addLayout(row)

        self.bar = QProgressBar()
        self.bar.setRange(0, 1000)
        self.bar.setTextVisible(False)
        lay.addWidget(self.bar)
        self.state_lbl = QLabel("Connect to a device to start")
        self.state_lbl.setObjectName("Muted")
        self.state_lbl.setWordWrap(True)
        lay.addWidget(self.state_lbl)

        self.step_btn.clicked.connect(lambda: self.start_requested.emit("step"))
        self.prbs_btn.clicked.connect(lambda: self.start_requested.emit("prbs"))
        self.abort_btn.clicked.connect(self.abort_requested.emit)
        self.set_state("idle", False)

    def set_state(self, state: str, connected: bool) -> None:
        idle = state == "idle"
        self.step_btn.setEnabled(connected and idle)
        self.prbs_btn.setEnabled(connected and idle)
        self.abort_btn.setEnabled(connected and state == "running")
        set_prop(self.bar, "state", state)
        if not connected:
            self.bar.setValue(0)
            self.state_lbl.setText("Connect to a device to start")
        elif idle:
            self.state_lbl.setText("Ready")
        elif state == "receiving":
            self.bar.setValue(1000)
            self.state_lbl.setText("Receiving data from the device...")

    def show_started(self, kind: str, total: float) -> None:
        self.bar.setValue(0)
        self.state_lbl.setText(f"Running {kind.upper()} test: 0.0 / {total:.1f} s")

    def show_progress(self, elapsed: float, total: float) -> None:
        self.bar.setValue(int(1000 * elapsed / total) if total > 0 else 0)
        self.state_lbl.setText(f"Running: {elapsed:.1f} / {total:.1f} s")

    def show_receiving(self, rows: int) -> None:
        self.state_lbl.setText(f"Receiving data: {rows} rows")

    def show_result(self, text: str) -> None:
        self.state_lbl.setText(text)
