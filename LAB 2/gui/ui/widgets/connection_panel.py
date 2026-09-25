"""Port selection and connect/disconnect."""
from __future__ import annotations

from PyQt6.QtCore import pyqtSignal
from PyQt6.QtWidgets import QComboBox, QGridLayout, QHBoxLayout, QLabel, QWidget
from serial.tools import list_ports

from core.serial_worker import SIM_PORT
from .common import make_button, make_card, set_prop

BAUDS = ["115200", "9600", "57600", "230400", "460800", "921600"]


class ConnectionPanel(QWidget):
    connect_requested = pyqtSignal(str, int)
    disconnect_requested = pyqtSignal()

    def __init__(self) -> None:
        super().__init__()
        card, lay = make_card("Connection")
        outer = QHBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.addWidget(card)

        grid = QGridLayout()
        grid.setHorizontalSpacing(8)
        grid.setVerticalSpacing(8)
        self.port = QComboBox()
        self.port.setSizeAdjustPolicy(QComboBox.SizeAdjustPolicy.AdjustToMinimumContentsLengthWithIcon)
        self.port.setMinimumContentsLength(10)
        self.port.setToolTip("Serial port of the ESP32 (or the built-in simulated device)")
        self.refresh_btn = make_button("Refresh", "ghost", "Rescan serial ports (F5)")
        self.baud = QComboBox()
        self.baud.addItems(BAUDS)
        self.baud.setToolTip("Baud rate; the firmware uses 115200")
        grid.addWidget(QLabel("Port"), 0, 0)
        grid.addWidget(self.port, 0, 1)
        grid.addWidget(self.refresh_btn, 0, 2)
        grid.addWidget(QLabel("Baud"), 1, 0)
        grid.addWidget(self.baud, 1, 1, 1, 2)
        grid.setColumnStretch(1, 1)
        lay.addLayout(grid)

        row = QHBoxLayout()
        self.dot = QLabel()
        self.dot.setObjectName("Dot")
        set_prop(self.dot, "conn", "off")
        self.status = QLabel("Disconnected")
        self.status.setObjectName("Muted")
        self.status.setWordWrap(True)
        self.connect_btn = make_button("Connect", "primary", "Open the selected port (Ctrl+K)")
        row.addWidget(self.dot)
        row.addWidget(self.status, 1)
        row.addWidget(self.connect_btn)
        lay.addLayout(row)

        self.refresh_btn.clicked.connect(self.refresh_ports)
        self.connect_btn.clicked.connect(self._toggle)
        self._connected = False
        self.refresh_ports()

    def refresh_ports(self) -> None:
        current = self.port.currentData()
        self.port.clear()
        for p in sorted(list_ports.comports(), key=lambda p: p.device):
            self.port.addItem(f"{p.device}  -  {p.description}", p.device)
        self.port.addItem("Simulated device (4x speed)", SIM_PORT)
        idx = self.port.findData(current) if current else -1
        self.port.setCurrentIndex(idx if idx >= 0 else 0)

    def select_simulated(self) -> None:
        self.port.setCurrentIndex(self.port.findData(SIM_PORT))

    def toggle(self) -> None:
        if self.connect_btn.isEnabled():
            self._toggle()

    def _toggle(self) -> None:
        if self._connected:
            self.disconnect_requested.emit()
        else:
            self.set_busy()
            self.connect_requested.emit(self.port.currentData(), int(self.baud.currentText()))

    def set_busy(self) -> None:
        set_prop(self.dot, "conn", "busy")
        self.status.setText("Connecting...")
        self.connect_btn.setEnabled(False)

    def set_connected(self, connected: bool, port: str = "") -> None:
        self._connected = connected
        set_prop(self.dot, "conn", "on" if connected else "off")
        self.status.setText(f"Connected to {port}" if connected else "Disconnected")
        self.connect_btn.setEnabled(True)
        self.connect_btn.setText("Disconnect" if connected else "Connect")
        for w in (self.port, self.baud, self.refresh_btn):
            w.setEnabled(not connected)
