"""Serial I/O worker. Lives in its own QThread; the UI only talks to it via signals/slots."""
from __future__ import annotations

import time

import serial
from PyQt6.QtCore import QObject, QTimer, pyqtSignal, pyqtSlot

from .models import Run
from .parser import StreamParser
from .simulator import SimulatedSerial, run_duration_s

SIM_PORT = "simulated"
POLL_MS = 10
PROBE_DELAY_MS = 1500        # ESP32 resets on port open; give it time to boot
PROBE_REPLY_S = 2.5
START_GRACE_S = 8.0          # extra wait beyond the nominal run time before "no response"
STALL_S = 4.0                # silence allowed while receiving the dump
ABORT_WAIT_S = 2.0

IDLE, RUNNING, RECEIVING = "idle", "running", "receiving"


class SerialWorker(QObject):
    connected = pyqtSignal(str)
    disconnected = pyqtSignal()
    log_batch = pyqtSignal(list)              # [(direction, text)], direction in tx/rx/sys
    state_changed = pyqtSignal(str)
    run_started = pyqtSignal(str, float)      # kind, nominal seconds
    progress = pyqtSignal(float, float)       # elapsed, total
    receiving = pyqtSignal(int)               # rows received so far
    run_finished = pyqtSignal(str, object)    # kind, Run
    run_aborted = pyqtSignal()
    info = pyqtSignal(dict)
    error = pyqtSignal(str, str)              # code, technical detail (for the log only)

    def __init__(self) -> None:
        super().__init__()
        self._ser = None
        self._timer: QTimer | None = None
        self._parser = StreamParser()
        self._buf = b""
        self._state = IDLE
        self._kind = ""
        self._total = 0.0
        self._t_start = 0.0
        self._last_rx = 0.0
        self._abort_deadline = 0.0
        self._info_seen = False
        self._scale = 1.0
        self._last_progress = 0.0

    # ------------------------------------------------------------ connection
    @pyqtSlot(str, int)
    def open_port(self, port: str, baud: int) -> None:
        if self._ser is not None:
            self.close_port()
        try:
            if port == SIM_PORT:
                self._ser = SimulatedSerial()
                self._scale = self._ser.time_scale
            else:
                self._ser = serial.Serial(port, baud, timeout=0, write_timeout=1)
                self._scale = 1.0
        except serial.SerialException as exc:
            self._ser = None
            self.error.emit(_classify_open_error(str(exc)), str(exc))
            return
        except (OSError, ValueError) as exc:
            self._ser = None
            self.error.emit("open_failed", str(exc))
            return
        self._buf = b""
        self._parser.reset()
        self._set_state(IDLE)
        self._info_seen = False
        if self._timer is None:
            self._timer = QTimer(self)
            self._timer.setInterval(POLL_MS)
            self._timer.timeout.connect(self._poll)
        self._timer.start()
        self.connected.emit(port)
        self._sys(f"Opened {port} @ {baud}")
        QTimer.singleShot(PROBE_DELAY_MS if port != SIM_PORT else 200, self._probe)

    @pyqtSlot()
    def close_port(self) -> None:
        if self._timer is not None:
            self._timer.stop()
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None
            self._sys("Port closed")
            self.disconnected.emit()
        self._set_state(IDLE)

    def _probe(self) -> None:
        if self._ser is None or self._state != IDLE:
            return
        self._write(b"i")
        QTimer.singleShot(int(PROBE_REPLY_S * 1000), self._probe_check)

    def _probe_check(self) -> None:
        if self._ser is not None and not self._info_seen and self._state == IDLE:
            self.error.emit("no_response", "no reply to 'i'")

    # ------------------------------------------------------------- commands
    @pyqtSlot(str)
    def start_test(self, kind: str) -> None:
        if self._ser is None or self._state != IDLE or kind not in ("step", "prbs"):
            return
        self._parser.reset()
        self._kind = kind
        self._total = run_duration_s(kind) / self._scale
        self._t_start = self._last_rx = time.monotonic()
        self._write(b"s" if kind == "step" else b"p")
        self._set_state(RUNNING)
        self.run_started.emit(kind, self._total)

    @pyqtSlot()
    def abort(self) -> None:
        if self._ser is None or self._state != RUNNING:
            return
        self._write(b"x")
        self._abort_deadline = time.monotonic() + ABORT_WAIT_S

    @pyqtSlot()
    def request_info(self) -> None:
        if self._ser is not None and self._state == IDLE:
            self._write(b"i")

    # ----------------------------------------------------------- internals
    def _write(self, data: bytes) -> None:
        try:
            self._ser.write(data)
            self.log_batch.emit([("tx", data.decode())])
        except (serial.SerialException, OSError) as exc:
            self._lost(str(exc))

    def _sys(self, text: str) -> None:
        self.log_batch.emit([("sys", text)])

    def _set_state(self, state: str) -> None:
        self._state = state
        self._abort_deadline = 0.0
        self.state_changed.emit(state)

    def _lost(self, detail: str) -> None:
        self.error.emit("lost", detail)
        self.close_port()

    def _poll(self) -> None:
        if self._ser is None:
            return
        now = time.monotonic()
        try:
            n = self._ser.in_waiting
            data = self._ser.read(n) if n else b""
        except (serial.SerialException, OSError) as exc:
            self._lost(str(exc))
            return
        if data:
            self._last_rx = now
            self._buf += data
            self._consume_lines()
        if self._state == RUNNING:
            self._tick_running(now)
        elif self._state == RECEIVING and now - self._last_rx > STALL_S:
            self._fail("timeout", "data transfer stalled")

    def _consume_lines(self) -> None:
        *lines, self._buf = self._buf.split(b"\n")
        batch = []
        for raw in lines:
            text = raw.decode("ascii", errors="replace").rstrip("\r")
            batch.append(("rx", text))
            if self._handle(self._parser.feed_line(text)):
                break
        if batch:
            self.log_batch.emit(batch)
        if self._state == RECEIVING:
            self.receiving.emit(len(self._parser.rows))

    def _handle(self, ev) -> bool:
        """Process a parser event. Returns True when the run ended."""
        if ev is None:
            return False
        kind, payload = ev
        if kind == "info":
            self._info_seen = True
            self.info.emit(payload)
        elif kind == "header" and self._state == RUNNING:
            self._set_state(RECEIVING)
        elif kind == "aborted":
            self._set_state(IDLE)
            self.run_aborted.emit()
            return True
        elif kind == "done" and self._state in (RUNNING, RECEIVING):
            self._finish()
            return True
        return False

    def _finish(self) -> None:
        p = self._parser
        kind = self._kind
        if not p.in_data or not p.rows:
            self._fail("malformed", f"no usable rows (bad={p.bad})")
            return
        run = Run.from_rows(kind, p.rows, p.bad)
        self._set_state(IDLE)
        self.run_finished.emit(kind, run)

    def _fail(self, code: str, detail: str) -> None:
        self._set_state(IDLE)
        self.error.emit(code, detail)

    def _tick_running(self, now: float) -> None:
        elapsed = now - self._t_start
        if now - self._last_progress >= 0.1:
            self._last_progress = now
            self.progress.emit(min(elapsed, self._total), self._total)
        if self._abort_deadline and now > self._abort_deadline:
            self._set_state(IDLE)
            self.run_aborted.emit()
        elif elapsed > self._total + START_GRACE_S and self._last_rx <= self._t_start:
            self._fail("no_response", "no data after run time elapsed")
        elif elapsed > self._total + START_GRACE_S * 3:
            self._fail("timeout", "run never completed")


def _classify_open_error(msg: str) -> str:
    low = msg.lower()
    if "access is denied" in low or "permission" in low or "busy" in low or "in use" in low:
        return "port_busy"
    if "filenotfound" in low or "could not find" in low or "no such" in low or "cannot find" in low:
        return "port_missing"
    return "open_failed"
