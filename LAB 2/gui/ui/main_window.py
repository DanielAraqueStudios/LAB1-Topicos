"""Main window: wires the widgets to the serial worker thread and the session."""
from __future__ import annotations

from pathlib import Path

from PyQt6.QtCore import QMetaObject, Qt, QThread, pyqtSignal
from PyQt6.QtGui import QAction, QKeySequence, QShortcut
from PyQt6.QtWidgets import (QApplication, QFileDialog, QFrame, QHBoxLayout, QLabel, QMainWindow,
                             QScrollArea, QSplitter, QVBoxLayout, QWidget)

from core.serial_worker import SerialWorker
from core.session import Session
from core.storage import DataFormatError
from .messages import friendly
from .theme import build_qss
from .widgets.common import make_button, set_prop
from .widgets.connection_panel import ConnectionPanel
from .widgets.log_console import LogConsole
from .widgets.plots_panel import PlotsPanel
from .widgets.quality_panel import QualityPanel
from .widgets.runs_panel import RunsPanel
from .widgets.test_panel import TestPanel


class MainWindow(QMainWindow):
    _open_port = pyqtSignal(str, int)
    _close_port = pyqtSignal()
    _start_test = pyqtSignal(str)
    _abort = pyqtSignal()

    def __init__(self, theme: str = "dark", data_dir: Path | None = None) -> None:
        super().__init__()
        self.setWindowTitle("Jib Crane Data Acquisition")
        self.resize(1360, 860)
        self.setMinimumSize(1000, 640)
        self.theme = theme
        self.session = Session(data_dir)
        self.connected = False
        self.state = "idle"

        self._build_ui()
        self._start_worker()
        self._bind_shortcuts()
        self.apply_theme(theme)

    # ------------------------------------------------------------------ UI
    def _build_ui(self) -> None:
        root = QWidget()
        root.setObjectName("Root")
        self.setCentralWidget(root)
        col = QVBoxLayout(root)
        col.setContentsMargins(16, 12, 16, 8)
        col.setSpacing(10)

        top = QHBoxLayout()
        title = QLabel("Plant characterization")
        title.setObjectName("Big")
        self.theme_btn = make_button("Light theme", "ghost", "Switch dark / light theme (Ctrl+T)")
        top.addWidget(title)
        top.addStretch(1)
        top.addWidget(self.theme_btn)
        col.addLayout(top)

        self.banner = QFrame()
        self.banner.setObjectName("Banner")
        bl = QHBoxLayout(self.banner)
        bl.setContentsMargins(12, 8, 8, 8)
        self.banner_title = QLabel()
        self.banner_title.setStyleSheet("font-weight: 700;")
        self.banner_text = QLabel()
        self.banner_text.setWordWrap(True)
        self.banner_text.setObjectName("Muted")
        close = make_button("Dismiss", "ghost")
        close.clicked.connect(self.banner.hide)
        bl.addWidget(self.banner_title)
        bl.addWidget(self.banner_text, 1)
        bl.addWidget(close)
        self.banner.hide()
        col.addWidget(self.banner)

        self.conn = ConnectionPanel()
        self.test = TestPanel()
        self.quality = QualityPanel()
        self.plots = PlotsPanel()
        self.runs = RunsPanel(self.session.folder)
        self.log = LogConsole()

        left = QWidget()
        ll = QVBoxLayout(left)
        ll.setContentsMargins(0, 0, 8, 0)
        ll.setSpacing(10)
        ll.addWidget(self.conn)
        ll.addWidget(self.test)
        ll.addWidget(self.quality, 1)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.Shape.NoFrame)
        scroll.setWidget(left)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        scroll.setFixedWidth(430)

        right = QSplitter(Qt.Orientation.Vertical)
        right.addWidget(self.plots)
        right.addWidget(self.runs)
        right.setStretchFactor(0, 3)
        right.setStretchFactor(1, 1)
        right.setChildrenCollapsible(False)
        rw = QWidget()
        rl = QVBoxLayout(rw)
        rl.setContentsMargins(8, 0, 0, 0)
        rl.setSpacing(6)
        rl.addWidget(right, 1)
        rl.addWidget(self.log)

        body = QHBoxLayout()
        body.setSpacing(0)
        body.addWidget(scroll)
        body.addWidget(rw, 1)
        col.addLayout(body, 1)
        self.statusBar().showMessage("Ready")

        self.theme_btn.clicked.connect(self.toggle_theme)
        self.conn.connect_requested.connect(self._open_port)
        self.conn.disconnect_requested.connect(self._close_port)
        self.test.start_requested.connect(self._start_test)
        self.test.abort_requested.connect(self._abort)
        self.runs.selection_changed.connect(self._on_selection)
        self.runs.visibility_changed.connect(self._refresh_plots)
        self.runs.save_requested.connect(self.save_selected)
        self.runs.load_requested.connect(self.load_file)
        self.runs.remove_requested.connect(self.remove_selected)
        self.runs.folder_requested.connect(self.choose_folder)

    def _bind_shortcuts(self) -> None:
        def sc(keys: str, fn) -> None:
            s = QShortcut(QKeySequence(keys), self)
            s.activated.connect(fn)

        sc("Ctrl+K", self.conn.toggle)
        sc("F5", self.conn.refresh_ports)
        sc("Ctrl+1", lambda: self.test.step_btn.isEnabled() and self._start_test.emit("step"))
        sc("Ctrl+2", lambda: self.test.prbs_btn.isEnabled() and self._start_test.emit("prbs"))
        sc("Esc", lambda: self.test.abort_btn.isEnabled() and self._abort.emit())
        sc("Ctrl+S", self.save_selected)
        sc("Ctrl+O", self.load_file)
        sc("Del", self.remove_selected)
        sc("Ctrl+T", self.toggle_theme)
        sc("Ctrl+L", self.log.toggle)
        sc("Ctrl+0", self.plots.fit)

    # -------------------------------------------------------------- worker
    def _start_worker(self) -> None:
        self.thread = QThread(self)
        self.worker = SerialWorker()
        self.worker.moveToThread(self.thread)
        w = self.worker
        self._open_port.connect(w.open_port)
        self._close_port.connect(w.close_port)
        self._start_test.connect(w.start_test)
        self._abort.connect(w.abort)
        w.connected.connect(self._on_connected)
        w.disconnected.connect(self._on_disconnected)
        w.log_batch.connect(self.log.append_batch)
        w.state_changed.connect(self._on_state)
        w.run_started.connect(self.test.show_started)
        w.progress.connect(self.test.show_progress)
        w.receiving.connect(self.test.show_receiving)
        w.run_finished.connect(self._on_run_finished)
        w.run_aborted.connect(lambda: self.test.show_result("Test aborted"))
        w.info.connect(lambda i: self.statusBar().showMessage(
            f"Device: Ts = {i['ts_us'] / 1000:g} ms, U = {i['u_pct']:g} %, limit = {i['max_pct']:g} %"))
        w.error.connect(self._on_error)
        self.thread.start()

    def _on_connected(self, port: str) -> None:
        self.connected = True
        self.conn.set_connected(True, "simulated device" if port == "simulated" else port)
        self.test.set_state(self.state, True)
        self.banner.hide()

    def _on_disconnected(self) -> None:
        self.connected = False
        self.conn.set_connected(False)
        self.test.set_state("idle", False)

    def _on_state(self, state: str) -> None:
        self.state = state
        self.test.set_state(state, self.connected)

    def _on_error(self, code: str, detail: str) -> None:
        title, text = friendly(code)
        self.log.append_batch([("sys", f"{code}: {detail}")])
        self.show_banner(title, text)
        if not self.connected:
            self.conn.set_connected(False)

    def show_banner(self, title: str, text: str, level: str = "error") -> None:
        self.banner_title.setText(title)
        self.banner_text.setText(text)
        set_prop(self.banner, "level", level)
        self.banner.show()

    # ---------------------------------------------------------------- runs
    def _on_run_finished(self, kind: str, run) -> None:
        row = self.session.add(run)
        self.runs.refresh(self.session.runs, select=row)
        q = run.quality.overall.upper()
        self.test.show_result(f"{run.name}: {run.n} samples, quality {q}")
        self.statusBar().showMessage(f"Received {run.name} ({run.n} samples). Not saved yet.")

    def _on_selection(self, row: int) -> None:
        if 0 <= row < len(self.session.runs):
            run = self.session.runs[row]
            self.quality.show_report(run.name, run.quality)
        else:
            self.quality.clear()
        self._refresh_plots()

    def _refresh_plots(self) -> None:
        sel = self.runs.current_row()
        items = [(r, self.runs.color_for(i), i == sel) for i, r in enumerate(self.session.runs)
                 if self.runs.is_visible(i)]
        self.plots.set_runs(items)

    def save_selected(self) -> None:
        row = self.runs.current_row()
        if row < 0:
            return
        try:
            path = self.session.save(row)
        except OSError as exc:
            self.log.append_batch([("sys", f"save_failed: {exc}")])
            self.show_banner(*friendly("save_failed"))
            return
        self.runs.refresh(self.session.runs, select=row)
        self.statusBar().showMessage(f"Saved {path}")

    def load_file(self) -> None:
        start = str(self.session.folder) if self.session.folder.is_dir() else str(Path.home())
        paths, _ = QFileDialog.getOpenFileNames(self, "Load run CSV", start, "CSV files (*.csv)")
        row = None
        for p in paths:
            try:
                row = self.session.load(Path(p))
            except (OSError, DataFormatError) as exc:
                self.log.append_batch([("sys", f"load_failed: {p}: {exc}")])
                self.show_banner(*friendly("load_failed"))
        if row is not None:
            self.runs.refresh(self.session.runs, select=row)

    def remove_selected(self) -> None:
        row = self.runs.current_row()
        if row < 0:
            return
        self.session.remove(row)
        self.runs.refresh(self.session.runs, select=min(row, len(self.session.runs) - 1))
        if not self.session.runs:
            self._on_selection(-1)

    def choose_folder(self) -> None:
        d = QFileDialog.getExistingDirectory(self, "Choose data folder", str(self.session.folder))
        if d:
            self.session.folder = Path(d)
            self.runs.set_folder(self.session.folder)

    # --------------------------------------------------------------- theme
    def toggle_theme(self) -> None:
        self.apply_theme("light" if self.theme == "dark" else "dark")

    def apply_theme(self, theme: str) -> None:
        self.theme = theme
        app = QApplication.instance()
        app.setStyleSheet(build_qss(theme))
        self.theme_btn.setText("Light theme" if theme == "dark" else "Dark theme")
        self.runs.set_theme(theme)
        self.plots.apply_theme(theme)
        self.runs.refresh(self.session.runs)

    # --------------------------------------------------------------- close
    def closeEvent(self, event) -> None:
        QMetaObject.invokeMethod(self.worker, "close_port", Qt.ConnectionType.BlockingQueuedConnection)
        self.thread.quit()
        self.thread.wait(2000)
        super().closeEvent(event)
