from __future__ import annotations

import pyqtgraph as pg
from PyQt6.QtWidgets import QGridLayout, QWidget

from ui import theme

WINDOW_S = 12.0
MAX_POINTS = 400


class AxisPlots(QWidget):
    def __init__(self, title: str, pos_unit: str, parent=None) -> None:
        super().__init__(parent)
        layout = QGridLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        self.pos_plot = pg.PlotWidget(title=f"{title} — posición")
        self.pwm_plot = pg.PlotWidget(title=f"{title} — PWM")
        self.err_plot = pg.PlotWidget(title=f"{title} — error")
        for p in (self.pos_plot, self.pwm_plot, self.err_plot):
            p.setBackground(theme.PLOT_BG)
            p.showGrid(x=True, y=True, alpha=0.3)
            p.getAxis("left").setPen(theme.PLOT_FG)
            p.getAxis("bottom").setPen(theme.PLOT_FG)

        self.pos_plot.setLabel("left", pos_unit)
        self.ref_curve = self.pos_plot.plot(pen=pg.mkPen(theme.REF_COLOR, width=2))
        self.meas_curve = self.pos_plot.plot(pen=pg.mkPen(theme.MEAS_COLOR, width=2))

        self.pwm_plot.setLabel("left", "PWM")
        self.pwm_curve = self.pwm_plot.plot(pen=pg.mkPen(theme.PWM_COLOR, width=2))

        self.err_plot.setLabel("left", pos_unit)
        self.err_curve = self.err_plot.plot(pen=pg.mkPen(theme.ERR_COLOR, width=2))

        layout.addWidget(self.pos_plot, 0, 0)
        layout.addWidget(self.pwm_plot, 1, 0)
        layout.addWidget(self.err_plot, 2, 0)

        self.t: list[float] = []
        self.ref: list[float] = []
        self.meas: list[float] = []
        self.pwm: list[float] = []
        self.err: list[float] = []

    def push(self, t: float, ref: float, meas: float, pwm: float, err: float) -> None:
        self.t.append(t)
        self.ref.append(ref)
        self.meas.append(meas)
        self.pwm.append(pwm)
        self.err.append(err)
        if len(self.t) > MAX_POINTS:
            self.t = self.t[-MAX_POINTS:]
            self.ref = self.ref[-MAX_POINTS:]
            self.meas = self.meas[-MAX_POINTS:]
            self.pwm = self.pwm[-MAX_POINTS:]
            self.err = self.err[-MAX_POINTS:]
        self.ref_curve.setData(self.t, self.ref)
        self.meas_curve.setData(self.t, self.meas)
        self.pwm_curve.setData(self.t, self.pwm)
        self.err_curve.setData(self.t, self.err)
        if self.t:
            self.pos_plot.setXRange(max(0, self.t[-1] - WINDOW_S), self.t[-1] + 0.5, padding=0)
            self.pwm_plot.setXRange(max(0, self.t[-1] - WINDOW_S), self.t[-1] + 0.5, padding=0)
            self.err_plot.setXRange(max(0, self.t[-1] - WINDOW_S), self.t[-1] + 0.5, padding=0)

    def clear(self) -> None:
        self.t, self.ref, self.meas, self.pwm, self.err = [], [], [], [], []
        for c in (self.ref_curve, self.meas_curve, self.pwm_curve, self.err_curve):
            c.setData([], [])
