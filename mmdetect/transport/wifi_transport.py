from __future__ import annotations

import logging
import socket
import time

from mmdetect.transport.base import AbstractTransport, TransportState

logger = logging.getLogger(__name__)
import platform
if platform.system() == "Windows":
    DEFAULT_PORT = 7000
elif platform.system() == "Darwin":
    DEFAULT_PORT = 5000
    # macOS-specific code
elif platform.system() == "Linux":
    DEFAULT_PORT = 5000
else:
    raise ValueError(f"Unsupported platform: {platform.system()}")
RECV_BUFFER = 1024
RECV_TIMEOUT_S = 0.5
STALE_TIMEOUT_S = 5.0


class WifiTransport(AbstractTransport):
    """Listens for UDP datagrams from the ESP32 in a dedicated QThread."""

    def __init__(
        self,
        bind_addr: str = "0.0.0.0",
        port: int = DEFAULT_PORT,
        parent=None,
    ):
        super().__init__(parent)
        self._bind_addr = bind_addr
        self._port = port
        self._sock: socket.socket | None = None
        self._remote_addr: tuple[str, int] | None = None

    # -- Configuration (only when disconnected) --

    @property
    def bind_addr(self) -> str:
        return self._bind_addr

    @bind_addr.setter
    def bind_addr(self, value: str) -> None:
        if self.is_connected:
            raise RuntimeError("Cannot change bind address while connected")
        self._bind_addr = value

    @property
    def port(self) -> int:
        return self._port

    @port.setter
    def port(self, value: int) -> None:
        if self.is_connected:
            raise RuntimeError("Cannot change port while connected")
        self._port = value

    @property
    def remote_addr(self) -> tuple[str, int] | None:
        """Address of the last ESP32 that sent us a packet (informational)."""
        return self._remote_addr

    # -- Transport interface --

    def open(self) -> None:
        if self.isRunning():
            return
        self._stop_requested = False
        self.start()

    def close(self) -> None:
        self._stop_requested = True
        self._cleanup()

    # -- QThread entry point --

    def run(self) -> None:
        try:
            self._bind()
        except OSError as exc:
            logger.error("Failed to bind UDP socket: %s", exc)
            self.error_occurred.emit(f"Bind failed: {exc}")
            self._set_state(TransportState.ERROR)
            self._cleanup()
            return

        self._set_state(TransportState.CONNECTED)
        logger.info("Listening on %s:%d (UDP)", self._bind_addr, self._port)

        try:
            self._read_loop()
        except Exception as exc:
            logger.exception("UDP transport error: %s", exc)
            self.error_occurred.emit(str(exc))

        self._cleanup()
        self._set_state(TransportState.DISCONNECTED)

    # -- Internal helpers --

    def _bind(self) -> None:
        self._set_state(TransportState.CONNECTING)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.settimeout(RECV_TIMEOUT_S)
        self._sock.bind((self._bind_addr, self._port))

    def _read_loop(self) -> None:
        assert self._sock is not None
        while not self._stop_requested:
            try:
                data, addr = self._sock.recvfrom(RECV_BUFFER)
            except socket.timeout:
                continue
            except OSError:
                if self._stop_requested:
                    break
                raise

            self._remote_addr = addr
            if data:
                self.bytes_received.emit(data)

    def _cleanup(self) -> None:
        if self._sock:
            try:
                self._sock.close()
            except Exception:
                pass
        self._sock = None
        self._remote_addr = None