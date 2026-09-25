import time

from PyQt6.QtWidgets import QApplication

from ui.main_window import MainWindow


def pump(app, cond, timeout):
    end = time.monotonic() + timeout
    while time.monotonic() < end and not cond():
        app.processEvents()
        time.sleep(0.01)
    return cond()


def test_simulated_step_abort_overlay(tmp_path):
    app = QApplication.instance() or QApplication([])
    win = MainWindow(data_dir=tmp_path)
    win.show()
    win.conn.select_simulated()
    win.conn._toggle()
    assert pump(app, lambda: win.connected, 3)
    assert pump(app, lambda: win.test.step_btn.isEnabled(), 3)

    win._start_test.emit("step")
    assert pump(app, lambda: win.state == "running", 2)
    assert pump(app, lambda: len(win.session.runs) == 1, 15)
    run = win.session.runs[0]
    assert run.name == "step_01" and run.n == 1600
    assert win.quality.overall.text() in ("PASS", "WARN", "FAIL")
    assert len(win.plots.plots[1].listDataItems()) == 1
    assert win.runs.table.rowCount() == 1

    # abort while running
    assert pump(app, lambda: win.test.step_btn.isEnabled(), 3)
    win._start_test.emit("prbs")
    assert pump(app, lambda: win.state == "running", 2)
    win._abort.emit()
    assert pump(app, lambda: win.state == "idle" and win.test.step_btn.isEnabled(), 4)
    assert len(win.session.runs) == 1

    # second run, overlay, save, theme
    win._start_test.emit("step")
    assert pump(app, lambda: len(win.session.runs) == 2, 15)
    assert len(win.plots.plots[0].listDataItems()) == 2
    win.save_selected()
    assert (tmp_path / "step_02.csv").exists()
    win.toggle_theme()
    win.plots.volts.setChecked(True)
    win.close()
