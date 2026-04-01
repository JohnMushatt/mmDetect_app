from __future__ import annotations

from abc import abstractmethod
from enum import Enum, auto

from PySide6.QtCore import QThread, Signal


class TransportState(Enum):
    DISCONNECTED = auto()
    CONNECTING = auto()
    CONNECTED = auto()
    RECONNECTING = auto()
    ERROR = auto()


class AbstractTransport(QThread):
    """Base class for all transport backends (serial, WiFi, etc.).

    Subclasses override run() to implement the read loop and must
    call bytes_received.emit(data) whenever raw bytes arrive.
    """

    bytes_received = Signal(bytes)
    state_changed = Signal(object)      # emits TransportState
    error_occurred = Signal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._state = TransportState.DISCONNECTED
        self._stop_requested = False

    @property
    def state(self) -> TransportState:
        return self._state

    @property
    def is_connected(self) -> bool:
        return self._state == TransportState.CONNECTED

    def _set_state(self, new_state: TransportState) -> None:
        if self._state != new_state:
            self._state = new_state
            self.state_changed.emit(new_state)

    @abstractmethod
    def open(self) -> None:
        """Begin the connection. Typically calls self.start() to launch the thread."""
        ...

    @abstractmethod
    def close(self) -> None:
        """Request a graceful shutdown of the connection."""
        ...

    def stop(self) -> None:
        """Signal the thread to stop and wait for it to finish."""
        self._stop_requested = True
        self.close()
        if self.isRunning():
            self.wait(3000)