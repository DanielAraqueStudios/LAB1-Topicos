from __future__ import annotations

import time

import serial
from PyQt6.QtCore import QObject, QTimer, pyqtSignal, pyqtSlot

from . import constants as C
from .cascade import CascadeController, CascadeOutput
from .protocol import format_command, is_error_line, parse_telemetry
from .simulator import SimulatedPlant

SIM_PORT = "simulated"
PROBE_DELAY_MS = 2000
WAIT_FIRST_MEASUREMENT_S = 3.0


class CascadeWorker(QObject):
    connected = pyqtSignal(str)
    disconnected = pyqtSignal()
    connect_failed = pyqtSignal(str)
    running_changed = pyqtSignal(bool)
    tick = pyqtSignal(float, float, float, float, float, float, float, float, float, float, float, float)
    alarm = pyqtSignal(str)
    log_batch = pyqtSignal(list)

    def __init__(self) -> None:
        super().__init__()
        self._ser = None
        self._sim_plant: SimulatedPlant | None = None
        self._is_sim = False
        self._cascade: CascadeController | None = None
        self._timer: QTimer | None = None
        self._running = False
        self._buf = b""
        self._last_t = 0.0

    @pyqtSlot(str, int, str)
    def open_port(self, port: str, baud: int, kind: str) -> None:
        self.close_port()
        self._cascade = CascadeController(kind)
        self._is_sim = port == SIM_PORT
        if self._is_sim:
            self._sim_plant = SimulatedPlant()
            self._cascade.seed_from_measurement(0.0, 0.0, 0.0)
            self.connected.emit(port)
            self.log_batch.emit([("sys", "Simulated plant ready")])
            self._start_timer()
            return
        try:
            self._ser = serial.Serial(port, baud, timeout=0, write_timeout=1)
        except (serial.SerialException, OSError, ValueError) as exc:
            self._ser = None
            self.connect_failed.emit(str(exc))
            self.log_batch.emit([("sys", f"open failed: {exc}")])
            return
        self.log_batch.emit([("sys", f"Opened {port} @ {baud}, waiting for ESP32 reset")])
        QTimer.singleShot(PROBE_DELAY_MS, self._after_reset)

    def _after_reset(self) -> None:
        if self._ser is None:
            return
        try:
            self._ser.reset_input_buffer()
            self._ser.write(b"START\n")
        except (serial.SerialException, OSError) as exc:
            self.connect_failed.emit(str(exc))
            return
        self.connected.emit(self._ser.port)
        self._start_timer()

    def _start_timer(self) -> None:
        if self._timer is None:
            self._timer = QTimer(self)
            self._timer.setInterval(int(C.SAMPLE_TIME * 1000))
            self._timer.timeout.connect(self._step)
        self._timer.start()

    @pyqtSlot()
    def close_port(self) -> None:
        if self._timer is not None:
            self._timer.stop()
        if self._ser is not None:
            try:
                self._ser.write(b"D0.0,U0,R0.0,V0\n")
                self._ser.write(b"STOP\n")
                self._ser.close()
            except Exception:
                pass
            self._ser = None
            self.disconnected.emit()
            self.log_batch.emit([("sys", "Port closed")])
        self._sim_plant = None
        self._running = False

    @pyqtSlot(str)
    def set_controller_kind(self, kind: str) -> None:
        if self._cascade is not None:
            self._cascade.set_kind(kind)

    @pyqtSlot(float)
    def set_linear_reference(self, value_cm: float) -> None:
        if self._cascade is not None:
            self._cascade.dist_ref = max(0.0, min(C.MAX_DIST_CM, value_cm))

    @pyqtSlot(float)
    def set_rotation_reference(self, value_deg: float) -> None:
        if self._cascade is not None:
            self._cascade.rot_ref = max(-C.MAX_ROT_DEG, min(C.MAX_ROT_DEG, value_deg))

    def _read_latest_telemetry(self):
        if self._ser is None:
            return None
        try:
            n = self._ser.in_waiting
            if n:
                self._buf += self._ser.read(n)
        except (serial.SerialException, OSError):
            return None
        latest = None
        if b"\n" in self._buf:
            *lines, self._buf = self._buf.split(b"\n")
            batch = []
            for raw in lines:
                text = raw.decode("ascii", errors="replace").strip()
                if not text:
                    continue
                batch.append(("rx", text))
                if is_error_line(text):
                    self.alarm.emit(text)
                    continue
                tel = parse_telemetry(text)
                if tel is not None:
                    latest = tel
            if batch:
                self.log_batch.emit(batch)
        return latest

    def _step(self) -> None:
        if self._cascade is None:
            return
        now = time.monotonic()
        if self._is_sim:
            out = self._cascade.tick()
            cmd = format_command(self._cascade.dist_ref_ramp, out.u_lin, self._cascade.rot_ref_ramp, out.pwm_rot)
            y, yr, yp, t = self._sim_plant.step(out.u_lin, out.pwm_rot, C.SAMPLE_TIME)
            self._cascade.update_measurement(y, yr, yp, C.SAMPLE_TIME)
            telemetry = f"{t:.3f},{self._cascade.dist_ref_ramp:.2f},{y:.2f},{round(out.u_lin)}," \
                        f"{self._cascade.rot_ref_ramp:.2f},{yr:.2f},{round(out.pwm_rot)},{yp:.2f}"
            self.log_batch.emit([("tx", cmd), ("rx", telemetry)])
            self._emit_tick(t, out, y, yr, yp)
            return

        tel = self._read_latest_telemetry()
        if tel is not None:
            dt = now - self._last_t
            self._cascade.update_measurement(tel.pos_lin_cm, tel.pos_rot_deg, tel.pos_pend_deg, dt)
            self._last_t = now
        else:
            self._cascade.mark_no_data()

        out = self._cascade.tick()
        try:
            cmd = format_command(self._cascade.dist_ref_ramp, out.u_lin, self._cascade.rot_ref_ramp, out.pwm_rot)
            self._ser.write((cmd + "\n").encode())
            self.log_batch.emit([("tx", cmd)])
        except (serial.SerialException, OSError) as exc:
            self.log_batch.emit([("sys", f"write failed: {exc}")])

        y = tel.pos_lin_cm if tel else self._cascade.lin.meas_y
        yr = tel.pos_rot_deg if tel else self._cascade.outer.meas_y
        yp = tel.pos_pend_deg if tel else self._cascade.inner.meas_y
        t = tel.t_s if tel else now
        self._emit_tick(t, out, y, yr, yp)

    def _emit_tick(self, t: float, out: CascadeOutput, y: float, yr: float, yp: float) -> None:
        self.tick.emit(
            t,
            self._cascade.dist_ref_ramp, y, out.u_lin, out.e_lin,
            self._cascade.rot_ref_ramp, yr, out.pwm_rot, out.e_rot,
            out.pend_ref, yp, out.e_pend,
        )
