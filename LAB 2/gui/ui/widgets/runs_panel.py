"""Session run table plus save/load/remove and output-folder controls."""
from __future__ import annotations

from pathlib import Path

from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtGui import QBrush, QColor
from PyQt6.QtWidgets import (QAbstractItemView, QHBoxLayout, QHeaderView, QLabel, QTableWidget,
                             QTableWidgetItem, QVBoxLayout, QWidget)

from core.models import Run
from ui.theme import PALETTES, RUN_COLORS
from .common import make_button, make_card

COLS = ["Show", "Name", "Type", "Samples", "Quality", "Saved"]
_TEXT = {"pass": "PASS", "warn": "WARN", "fail": "FAIL"}


class RunsPanel(QWidget):
    selection_changed = pyqtSignal(int)      # row or -1
    visibility_changed = pyqtSignal()
    save_requested = pyqtSignal()
    load_requested = pyqtSignal()
    remove_requested = pyqtSignal()
    folder_requested = pyqtSignal()

    def __init__(self, folder: Path) -> None:
        super().__init__()
        card, lay = make_card("Runs")
        outer = QHBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.addWidget(card)

        self.table = QTableWidget(0, len(COLS))
        self.table.setHorizontalHeaderLabels(COLS)
        self.table.verticalHeader().setVisible(False)
        self.table.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self.table.setSelectionMode(QAbstractItemView.SelectionMode.SingleSelection)
        self.table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.table.setAlternatingRowColors(True)
        self.table.setToolTip("Tick 'Show' on several runs to overlay them and check repeatability")
        hh = self.table.horizontalHeader()
        hh.setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        hh.setSectionResizeMode(0, QHeaderView.ResizeMode.ResizeToContents)
        lay.addWidget(self.table, 1)

        row = QHBoxLayout()
        self.save_btn = make_button("Save CSV", "primary", "Save the selected run to the folder (Ctrl+S)")
        self.load_btn = make_button("Load CSV...", "", "Open an existing CSV as a run (Ctrl+O)")
        self.remove_btn = make_button("Remove", "ghost", "Remove the selected run from the session (Del)")
        for b in (self.save_btn, self.load_btn, self.remove_btn):
            row.addWidget(b)
        row.addStretch(1)
        lay.addLayout(row)

        frow = QHBoxLayout()
        self.folder_lbl = QLabel()
        self.folder_lbl.setObjectName("Muted")
        self.folder_btn = make_button("Folder...", "ghost", "Choose where CSV files are saved")
        frow.addWidget(QLabel("Save to"))
        frow.addWidget(self.folder_lbl, 1)
        frow.addWidget(self.folder_btn)
        lay.addLayout(frow)
        self.set_folder(folder)

        self.table.itemSelectionChanged.connect(lambda: self.selection_changed.emit(self.current_row()))
        self.table.itemChanged.connect(self._on_item_changed)
        self.save_btn.clicked.connect(self.save_requested.emit)
        self.load_btn.clicked.connect(self.load_requested.emit)
        self.remove_btn.clicked.connect(self.remove_requested.emit)
        self.folder_btn.clicked.connect(self.folder_requested.emit)
        self._theme = "dark"
        self._updating = False
        self._update_buttons()

    # ------------------------------------------------------------------ API
    def set_folder(self, folder: Path) -> None:
        self.folder_lbl.setText(str(folder))
        self.folder_lbl.setToolTip(str(folder))

    def current_row(self) -> int:
        rows = self.table.selectionModel().selectedRows()
        return rows[0].row() if rows else -1

    def color_for(self, row: int) -> str:
        colors = RUN_COLORS[self._theme]
        return colors[row % len(colors)]

    def is_visible(self, row: int) -> bool:
        item = self.table.item(row, 0)
        return item is not None and item.checkState() == Qt.CheckState.Checked

    def set_theme(self, theme: str) -> None:
        self._theme = theme

    def refresh(self, runs: list[Run], select: int | None = None) -> None:
        """Rebuild rows from the session, keeping visibility flags by row where possible."""
        keep = [self.is_visible(r) for r in range(self.table.rowCount())]
        sel = self.current_row() if select is None else select
        self._updating = True
        self.table.setRowCount(len(runs))
        for r, run in enumerate(runs):
            chk = QTableWidgetItem()
            chk.setFlags(Qt.ItemFlag.ItemIsUserCheckable | Qt.ItemFlag.ItemIsEnabled)
            visible = keep[r] if r < len(keep) and select is None else True
            chk.setCheckState(Qt.CheckState.Checked if visible else Qt.CheckState.Unchecked)
            chk.setForeground(QBrush(QColor(self.color_for(r))))
            self.table.setItem(r, 0, chk)
            q = run.quality.overall if run.quality else None
            cells = [run.name, run.kind.upper(), str(run.n), _TEXT.get(q, "-"), "yes" if run.saved else "no"]
            for c, text in enumerate(cells, start=1):
                item = QTableWidgetItem(text)
                if c == 4 and q:
                    pal = PALETTES[self._theme]
                    item.setForeground(QBrush(QColor(pal[{"pass": "ok", "warn": "warn", "fail": "danger"}[q]])))
                self.table.setItem(r, c, item)
        self._updating = False
        if 0 <= (sel if sel is not None else -1) < len(runs):
            self.table.selectRow(sel)
        self._update_buttons()
        self.visibility_changed.emit()

    def _on_item_changed(self, item: QTableWidgetItem) -> None:
        if not self._updating and item.column() == 0:
            self.visibility_changed.emit()

    def _update_buttons(self) -> None:
        has = self.table.rowCount() > 0
        self.save_btn.setEnabled(has)
        self.remove_btn.setEnabled(has)
