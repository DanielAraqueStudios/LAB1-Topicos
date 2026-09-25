"""Three linked pyqtgraph plots (u, jib, cable) that overlay any number of runs."""
from __future__ import annotations

import pyqtgraph as pg
from PyQt6.QtWidgets import QCheckBox, QHBoxLayout, QVBoxLayout, QWidget

from core.models import Run, counts_to_volts
from ui.theme import PALETTES
from .common import make_button, make_card


class PlotsPanel(QWidget):
    def __init__(self) -> None:
        super().__init__()
        card, lay = make_card("Signals")
        outer = QHBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.addWidget(card)

        bar = QHBoxLayout()
        self.volts = QCheckBox("Show in volts")
        self.volts.setToolTip("Convert ADC counts to volts (raw / 4095 x 3.3 V)")
        self.fit_btn = make_button("Fit view", "ghost", "Reset zoom to show all data (Ctrl+0)")
        bar.addWidget(self.volts)
        bar.addStretch(1)
        bar.addWidget(self.fit_btn)
        lay.addLayout(bar)

        self.win = pg.GraphicsLayoutWidget()
        self.win.ci.layout.setSpacing(6)
        self.plots = []
        for i in range(3):
            p = self.win.addPlot(row=i, col=0)
            p.showGrid(x=True, y=True, alpha=0.35)
            p.setDownsampling(auto=True, mode="peak")
            p.setMenuEnabled(False)
            if i:
                p.setXLink(self.plots[0])
            if i < 2:
                p.getAxis("bottom").setStyle(showValues=False)
            self.plots.append(p)
        self.plots[2].setLabel("bottom", "Time", units="s")
        self.plots[0].addLegend(offset=(-8, 8), labelTextSize="9pt")
        lay.addWidget(self.win, 1)

        self._items: list[tuple[Run, str, bool]] = []
        self._theme = "dark"
        self.volts.toggled.connect(lambda _: self.redraw())
        self.fit_btn.clicked.connect(self.fit)
        self.apply_theme("dark")

    def apply_theme(self, theme: str) -> None:
        self._theme = theme
        pal = PALETTES[theme]
        self.win.setBackground(pal["plot_bg"])
        for p in self.plots:
            for ax in ("left", "bottom"):
                a = p.getAxis(ax)
                a.setPen(pg.mkPen(pal["grid"]))
                a.setTextPen(pg.mkPen(pal["plot_fg"]))
        self.redraw()

    def set_runs(self, items: list[tuple[Run, str, bool]]) -> None:
        """items: (run, color, is_selected) for every visible run."""
        self._items = items
        self.redraw()

    def fit(self) -> None:
        for p in self.plots:
            p.autoRange()
            p.enableAutoRange()

    def redraw(self) -> None:
        volts = self.volts.isChecked()
        pal = PALETTES[self._theme]
        unit = "V" if volts else "counts"
        labels = ("Input u (%)", f"Jib angle ({unit})", f"Cable angle ({unit})")
        for p, text in zip(self.plots, labels):
            p.clear()
            p.setLabel("left", text, color=pal["plot_fg"])
        legend = self.plots[0].legend
        if legend is not None:
            legend.clear()
        conv = counts_to_volts if volts else (lambda a: a)
        for run, color, selected in self._items:
            if run.n == 0:
                continue
            pen = pg.mkPen(color, width=2.2 if selected else 1.4)
            t = run.t_s
            series = (run.u_pct, conv(run.jib), conv(run.cable))
            for i, (p, y) in enumerate(zip(self.plots, series)):
                p.plot(t, y, pen=pen, name=run.name if i == 0 else None, skipFiniteCheck=True)
        self.fit()
