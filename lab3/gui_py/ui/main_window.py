from __future__ import annotations

from PyQt6.QtCore import QThread, Qt, pyqtSignal
from PyQt6.QtWidgets import (
    QComboBox, QGroupBox, QHBoxLayout, QLabel, QLineEdit, QMainWindow,
    QPushButton, QSlider, QTabWidget, QVBoxLayout, QWidget,
)

from core import constants as C
from core.serial_worker import SIM_PORT, CascadeWorker
from ui import theme
from ui.widgets.log_console import LogConsole
from ui.widgets.plots_panel import AxisPlots

try:
    import serial.tools.list_ports as list_ports
except ImportError:
    list_ports = None


class MainWindow(QMainWindow):
    _open_port = pyqtSignal(str, int, str)
    _close_port = pyqtSignal()
    _set_kind = pyqtSignal(str)
    _set_lin_ref = pyqtSignal(float)
    _set_rot_ref = pyqtSignal(float)

    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Grúa Pluma — Control en Cascada (Python/PyQt6)")
        self.resize(1400, 900)
        self.setStyleSheet(theme.QSS)

        self._thread = QThread(self)
        self._worker = CascadeWorker()
        self._worker.moveToThread(self._thread)
        self._thread.start()

        self._wire_worker()
        self._build_ui()
        self._connected = False

    def _wire_worker(self) -> None:
        w = self._worker
        self._open_port.connect(w.open_port)
        self._close_port.connect(w.close_port)
        self._set_kind.connect(w.set_controller_kind)
        self._set_lin_ref.connect(w.set_linear_reference)
        self._set_rot_ref.connect(w.set_rotation_reference)

        w.connected.connect(self._on_connected)
        w.disconnected.connect(self._on_disconnected)
        w.connect_failed.connect(self._on_connect_failed)
        w.tick.connect(self._on_tick)
        w.enc_lin.connect(self._on_enc_lin)
        w.alarm.connect(self._on_alarm)
        w.log_batch.connect(self._on_log_batch)

    def _build_ui(self) -> None:
        root = QWidget()
        self.setCentralWidget(root)
        outer = QHBoxLayout(root)

        outer.addWidget(self._build_control_panel(), 0)
        outer.addWidget(self._build_plots(), 1)

    def _build_control_panel(self) -> QWidget:
        panel = QWidget()
        panel.setFixedWidth(340)
        layout = QVBoxLayout(panel)

        conn_box = QGroupBox("Conexión")
        cl = QVBoxLayout(conn_box)
        self.port_combo = QComboBox()
        self.refresh_ports_btn = QPushButton("Actualizar")
        self.refresh_ports_btn.clicked.connect(self._refresh_ports)
        self._refresh_ports()
        self.baud_edit = QLineEdit(str(C.BAUD_RATE))
        self.controller_combo = QComboBox()
        self.controller_combo.addItems(["PID Discreto", "Lugar de Raíces (z)", "Oscilaciones Muertas", "Servosistema"])
        self.controller_combo.currentIndexChanged.connect(self._on_controller_changed)
        self.connect_btn = QPushButton("Conectar")
        self.connect_btn.setProperty("variant", "primary")
        self.connect_btn.clicked.connect(self._on_connect_clicked)
        port_row = QHBoxLayout()
        port_row.addWidget(self.port_combo)
        port_row.addWidget(self.refresh_ports_btn)
        cl.addWidget(QLabel("Puerto:"))
        cl.addLayout(port_row)
        cl.addWidget(QLabel("Baud:"))
        cl.addWidget(self.baud_edit)
        cl.addWidget(QLabel("Controlador (cascada):"))
        cl.addWidget(self.controller_combo)
        cl.addWidget(self.connect_btn)
        layout.addWidget(conn_box)

        lin_box = QGroupBox("Eje Lineal")
        ll = QVBoxLayout(lin_box)
        self.lin_value_lbl = QLabel("0.0 cm")
        self.lin_value_lbl.setObjectName("Value")
        self.lin_slider = QSlider(Qt.Orientation.Horizontal)
        self.lin_slider.setRange(0, int(C.MAX_DIST_CM * 10))
        self.lin_slider.valueChanged.connect(self._on_lin_slider)
        self.enc_lin_lbl = QLabel("Encoder lineal: -- cuentas")
        ll.addWidget(QLabel("Referencia de posición (cm):"))
        ll.addWidget(self.lin_slider)
        ll.addWidget(self.lin_value_lbl)
        ll.addWidget(self.enc_lin_lbl)
        layout.addWidget(lin_box)

        rot_box = QGroupBox("Eje de Rotación (cascada)")
        rl = QVBoxLayout(rot_box)
        self.rot_value_lbl = QLabel("0.0 °")
        self.rot_value_lbl.setObjectName("Value")
        self.rot_slider = QSlider(Qt.Orientation.Horizontal)
        self.rot_slider.setRange(-int(C.MAX_ROT_DEG * 10), int(C.MAX_ROT_DEG * 10))
        self.rot_slider.valueChanged.connect(self._on_rot_slider)
        self.pend_ref_lbl = QLabel("Ref. péndulo: 0.0 °")
        self.pend_meas_lbl = QLabel("Péndulo medido: 0.0 °")
        rl.addWidget(QLabel("Referencia de ángulo BASE (°):"))
        rl.addWidget(self.rot_slider)
        rl.addWidget(self.rot_value_lbl)
        rl.addWidget(self.pend_ref_lbl)
        rl.addWidget(self.pend_meas_lbl)
        layout.addWidget(rot_box)

        actions = QHBoxLayout()
        self.start_btn = QPushButton("INICIAR")
        self.start_btn.setProperty("variant", "primary")
        self.start_btn.clicked.connect(self._on_connect_clicked)
        self.stop_btn = QPushButton("DETENER")
        self.stop_btn.setProperty("variant", "danger")
        self.stop_btn.clicked.connect(self._on_disconnect_clicked)
        self.stop_btn.setEnabled(False)
        actions.addWidget(self.start_btn)
        actions.addWidget(self.stop_btn)
        layout.addLayout(actions)

        self.status_lbl = QLabel("Estado: DETENIDO")
        self.status_lbl.setObjectName("Status")
        self.status_lbl.setWordWrap(True)
        layout.addWidget(self.status_lbl)
        layout.addStretch(1)
        return panel

    def _build_plots(self) -> QWidget:
        tabs = QTabWidget()
        self.plots_lin = AxisPlots("Lineal", "cm")
        self.plots_rot = AxisPlots("Base (externo)", "deg")
        self.plots_pend = AxisPlots("Péndulo (interno)", "deg")
        self.log_console = LogConsole()
        tabs.addTab(self.plots_lin, "Lineal")
        tabs.addTab(self.plots_rot, "Rotación (base)")
        tabs.addTab(self.plots_pend, "Péndulo")
        tabs.addTab(self.log_console, "Log serial")
        return tabs

    def _refresh_ports(self) -> None:
        self.port_combo.clear()
        self.port_combo.addItem(SIM_PORT)
        if list_ports is not None:
            for p in list_ports.comports():
                self.port_combo.addItem(p.device)

    def _current_kind(self) -> str:
        return C.CONTROLLERS[self.controller_combo.currentIndex()]

    def _on_controller_changed(self) -> None:
        if self._connected:
            self._set_kind.emit(self._current_kind())
            self.plots_lin.clear()
            self.plots_rot.clear()
            self.plots_pend.clear()

    def _on_lin_slider(self, raw: int) -> None:
        value = raw / 10.0
        self.lin_value_lbl.setText(f"{value:.1f} cm")
        if self._connected:
            self._set_lin_ref.emit(value)

    def _on_rot_slider(self, raw: int) -> None:
        value = raw / 10.0
        self.rot_value_lbl.setText(f"{value:.1f} °")
        if self._connected:
            self._set_rot_ref.emit(value)

    def _on_connect_clicked(self) -> None:
        if self._connected:
            return
        port = self.port_combo.currentText()
        try:
            baud = int(self.baud_edit.text())
        except ValueError:
            baud = C.BAUD_RATE
        self.plots_lin.clear()
        self.plots_rot.clear()
        self.plots_pend.clear()
        self._open_port.emit(port, baud, self._current_kind())

    def _on_disconnect_clicked(self) -> None:
        self._close_port.emit()

    def _on_connected(self, port: str) -> None:
        self._connected = True
        self.connect_btn.setEnabled(False)
        self.stop_btn.setEnabled(True)
        self.status_lbl.setText(f"Estado: CORRIENDO ({port})")

    def _on_disconnected(self) -> None:
        self._connected = False
        self.connect_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)
        self.status_lbl.setText("Estado: DETENIDO")

    def _on_connect_failed(self, detail: str) -> None:
        self._connected = False
        self.status_lbl.setText(f"Error de conexión: {detail}")

    def _on_alarm(self, text: str) -> None:
        self.status_lbl.setText(f"Estado: CORRIENDO  |  {text}")

    def _on_enc_lin(self, count: int) -> None:
        self.enc_lin_lbl.setText(f"Encoder lineal: {count} cuentas")

    def _on_log_batch(self, items: list) -> None:
        self.log_console.append_batch(items)

    def _on_tick(self, t, ref_lin, pos_lin, pwm_lin, e_lin,
                 ref_rot, pos_rot, pwm_rot, e_rot,
                 pend_ref, pos_pend, e_pend) -> None:
        self.plots_lin.push(t, ref_lin, pos_lin, pwm_lin, e_lin)
        self.plots_rot.push(t, ref_rot, pos_rot, pwm_rot, e_rot)
        self.plots_pend.push(t, pend_ref, pos_pend, pwm_rot, e_pend)
        self.pend_ref_lbl.setText(f"Ref. péndulo: {pend_ref:.1f} °")
        self.pend_meas_lbl.setText(f"Péndulo medido: {pos_pend:.1f} °")

    def closeEvent(self, event) -> None:
        self._close_port.emit()
        self._thread.quit()
        self._thread.wait(1000)
        super().closeEvent(event)
