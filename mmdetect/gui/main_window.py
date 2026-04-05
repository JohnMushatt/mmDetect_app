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

from collections import deque
import math
TRAIL_LENGTH = 100 # Number of historical frames to keep track of
MAX_STALE_FRAMES = 20 # Prune tracks if no update for this many frames
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
        self._refresh_btn.clicked.connect(self._refresh_ports)
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


        self._tracks: dict[int, deque] = {} # Frame ID -> deque of targets
        self._track_misses: dict[int, int] = {} # Track ID -> number of frames since last update
        self._next_track_id = 0
        self._max_association_dist_mm = 500

        # Trail scatter
        self._trail_scatter = pg.ScatterPlotItem(size=6,
         pen=pg.mkPen(color=(0, 120, 255, 100), width=1),
         brush=pg.mkBrush(color=(0, 120, 255, 100)))
        self._plot.addItem(self._trail_scatter)

        # Direction arrows per target
        self._arrow_items: dict[pg.ArrowItem] = [] # Target ID -> ArrowItem
    # Trail tracking
    def _associate_targets(self, detections: list[tuple[float, float]]) -> None:
        """Match new detections to existing tracks via nearest-neighbor, or create new tracks."""
        used_tracks = set()
        used_detections = set()
        # Build cost matrix: (track_id, det_idx) -> distance
        assignments = []
        for det_idx, (dx, dy) in enumerate(detections):
            for track_id, trail in self._tracks.items():
                last_x, last_y = trail[-1]
                dist = math.hypot(dx - last_x, dy - last_y)
                assignments.append((dist, track_id, det_idx))
        assignments.sort()  # greedy nearest-neighbor
        for dist, track_id, det_idx in assignments:
            if track_id in used_tracks or det_idx in used_detections:
                continue
            if dist > self._max_association_dist_mm:
                continue
            self._tracks[track_id].append(detections[det_idx])
            used_tracks.add(track_id)
            used_detections.add(det_idx)
        # Create new tracks for unmatched detections
        for det_idx, (dx, dy) in enumerate(detections):
            if det_idx not in used_detections:
                self._tracks[self._next_track_id] = deque(maxlen=TRAIL_LENGTH)
                self._tracks[self._next_track_id].append((dx, dy))
                self._track_misses[self._next_track_id] = 0
                self._next_track_id += 1
        # Optionally: prune stale tracks that haven't been updated
        # (track_ids not in used_tracks for N consecutive frames)
        stale_ids = []
        for track_id in list(self._tracks.keys()):
            if track_id in used_tracks:
                self._track_misses[track_id] = 0
            else:
                self._track_misses[track_id] = self._track_misses.get(track_id, 0) + 1
                if self._track_misses[track_id] >= MAX_STALE_FRAMES:
                    stale_ids.append(track_id)
        for track_id in stale_ids:
            del self._tracks[track_id]
            del self._track_misses[track_id]


    def _update_trail_display(self) -> None:
        """Render trails with fading opacity and direction arrows."""
        spots = []
        # Per-track color palette
        colors = [
            (0, 120, 255),   # blue
            (255, 80, 80),    # red
            (80, 220, 80),    # green
        ]
        # Clear old arrows
        for arrow in self._arrow_items:
            self._plot.removeItem(arrow)
        self._arrow_items.clear()
        current_points_x = []
        current_points_y = []
        for i, (track_id, trail) in enumerate(self._tracks.items()):
            r, g, b = colors[i % len(colors)]
            trail_list = list(trail)
            for j, (x, y) in enumerate(trail_list):
                # Opacity fades from dim (oldest) to bright (newest)
                alpha = int(40 + 200 * (j / max(len(trail_list) - 1, 1)))
                spots.append({
                    'pos': (x, y),
                    'size': 4 + 6 * (j / max(len(trail_list) - 1, 1)),
                    'brush': pg.mkBrush(r, g, b, alpha),
                })
            # Current position (last point)
            if trail_list:
                cx, cy = trail_list[-1]
                current_points_x.append(cx)
                current_points_y.append(cy)
            # Direction arrow from second-to-last to last point
            if len(trail_list) >= 2:
                x0, y0 = trail_list[-2]
                x1, y1 = trail_list[-1]
                angle = math.degrees(math.atan2(y1 - y0, x1 - x0))
                arrow = pg.ArrowItem(
                    pos=(x1, y1),
                    angle=-angle + 180,  # pyqtgraph angle convention
                    tipAngle=30,
                    headLen=15,
                    brush=pg.mkBrush(r, g, b, 220),
                    pen=pg.mkPen(r, g, b, 220),
                )
                self._plot.addItem(arrow)
                self._arrow_items.append(arrow)
        self._trail_scatter.setData(spots)
        self._scatter.setData(current_points_x, current_points_y)
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
                directions = [(t.x_mm, t.y_mm) for t in valid]

                #x = np.array([t.x_mm for t in valid])
                #y = np.array([t.y_mm for t in valid])
                self._log.append(f"Frame ID: {frame.frame_id}, Timestamp: {frame.timestamp_ms}, Targets: {len(valid)}")
                self._associate_targets(directions)
                self._update_trail_display()
                #self._scatter.setData(x, y)
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