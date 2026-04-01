import logging
import pyqtgraph as pg
import numpy as np
from PySide6.QtWidgets import (
    QMainWindow, QLabel, QVBoxLayout, QWidget, QPushButton, QHBoxLayout,
    QComboBox, QTextEdit, QLineEdit, QStackedWidget, QSpinBox,
)
from PySide6.QtCore import Slot
import mmdetect.models.target as target
from mmdetect.transport import (
    AbstractTransport, TransportState, WifiTransport,
)
from mmdetect.transport.wifi_transport import DEFAULT_PORT
logger = logging.getLogger(__name__)


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("mmDetect - Radar Target Tracker")
        self.setMinimumSize(800, 600)

        self._transport: AbstractTransport | None = None

        # -- UI --
        central = QWidget()
        root_layout = QVBoxLayout(central)

        controls = QHBoxLayout()

        # Transport type selector
        self._type_combo = QComboBox()
        self._type_combo.addItems(["WiFi"])
        self._type_combo.currentIndexChanged.connect(self._on_transport_type_changed)
        controls.addWidget(self._type_combo)

        # Stacked config: page 0 = serial, page 1 = WiFi
        self._config_stack = QStackedWidget()

        # Serial config
        """
        serial_config = QWidget()
        serial_layout = QHBoxLayout(serial_config)
        serial_layout.setContentsMargins(0, 0, 0, 0)
        self._port_combo = QComboBox()
        self._port_combo.setMinimumWidth(180)
        #self._refresh_ports()
        serial_layout.addWidget(self._port_combo)
        self._refresh_btn = QPushButton("Refresh")
#        self._refresh_btn.clicked.connect(self._refresh_ports)
        serial_layout.addWidget(self._refresh_btn)
        self._config_stack.addWidget(serial_config)
        """
        # WiFi config
        wifi_config = QWidget()
        wifi_layout = QHBoxLayout(wifi_config)
        wifi_layout.setContentsMargins(0, 0, 0, 0)
        wifi_layout.addWidget(QLabel("Port:"))
        self._udp_port_spin = QSpinBox()
        self._udp_port_spin.setRange(1024, 65535)
        self._udp_port_spin.setValue(DEFAULT_PORT)
        wifi_layout.addWidget(self._udp_port_spin)
        self._config_stack.addWidget(wifi_config)

        controls.addWidget(self._config_stack)

        self._connect_btn = QPushButton("Connect")
        self._connect_btn.clicked.connect(self._toggle_connection)
        controls.addWidget(self._connect_btn)

        controls.addStretch()
        root_layout.addLayout(controls)

        self._log = QTextEdit()
        self._log.setReadOnly(True)
        self._log.setFontFamily("monospace")
        root_layout.addWidget(self._log)

        self._status_label = QLabel("Disconnected")
        root_layout.addWidget(self._status_label)

        self.setCentralWidget(central)


        # Real-time target display
        self._plot = pg.PlotWidget(title="Target Display")
        self._plot.setLabel("bottom", "X", units="mm")
        self._plot.setLabel("left", "Y", units="mm")
        self._plot.setAspectLocked(True)
        self._scatter =pg.ScatterPlotItem(size=10, pen=pg.mkPen(None), brush=pg.mkBrush(color=(0, 120, 255, 200)))
        self._plot.addItem(self._scatter)
        root_layout.addWidget(self._plot)

    # -- Transport management --

    def _create_transport(self) -> AbstractTransport:
        """Instantiate the transport selected by the dropdown."""
        if self._type_combo.currentText() == "Serial":
            port = self._port_combo.currentText()
            if not port:
                raise ValueError("No serial port selected")
            #return SerialTransport(port=port, parent=self)
            raise ValueError("Serial transport not supported")
        else:
            return WifiTransport(port=self._udp_port_spin.value(), parent=self)

    def _wire_transport(self, transport: AbstractTransport) -> None:
        transport.bytes_received.connect(self._on_bytes_received)
        transport.state_changed.connect(self._on_state_changed)
        transport.error_occurred.connect(self._on_error)

    def _teardown_transport(self) -> None:
        if self._transport is not None:
            self._transport.stop()
            self._transport.deleteLater()
            self._transport = None

    # -- Slots --

    @Slot(int)
    def _on_transport_type_changed(self, index: int):
        self._config_stack.setCurrentIndex(index)
    """
    @Slot()
    def _refresh_ports(self):
        self._port_combo.clear()
        for port in w.available_ports():
            self._port_combo.addItem(port)
    """
    @Slot()
    def _toggle_connection(self):
        if self._transport and (self._transport.is_connected or self._transport.isRunning()):
            self._teardown_transport()
            return

        try:
            self._transport = self._create_transport()
        except ValueError as exc:
            self._status_label.setText(str(exc))
            return

        self._wire_transport(self._transport)
        self._transport.open()

    @Slot(bytes)
    def _on_bytes_received(self, data: bytes):
        hex_str = data.hex(" ")
        self._log.append(hex_str)
        try:
            frame = target.TargetFrame.from_bytes(data)
            valid = [t for t in frame.targets if t.is_valid]
            if valid:
                x = np.array([t.x_mm for t in valid])
                y = np.array([t.y_mm for t in valid])
                self._log.append(f"Frame ID: {frame.frame_id}, Timestamp: {frame.timestamp_ms}, Targets: {len(valid)}")

                self._scatter.setData(x, y)
        except ValueError as exc:
            self._log.append(f"[ERROR] {exc}")
       # self._log.append(f"Target frame: {target_frame}")

    @Slot(object)
    def _on_state_changed(self, state: TransportState):
        self._status_label.setText(state.name)
        if state == TransportState.CONNECTED:
            self._connect_btn.setText("Disconnect")
            self._type_combo.setEnabled(False)
        else:
            self._connect_btn.setText("Connect")
            self._type_combo.setEnabled(True)
        logger.info("Transport state: %s", state.name)

    @Slot(str)
    def _on_error(self, message: str):
        self._log.append(f"[ERROR] {message}")
        logger.error("Transport error: %s", message)

    # -- Lifecycle --

    def closeEvent(self, event):
        self._teardown_transport()
        super().closeEvent(event)