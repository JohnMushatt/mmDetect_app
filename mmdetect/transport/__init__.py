from mmdetect.transport.base import AbstractTransport, TransportState
from mmdetect.transport.wifi_transport import WifiTransport
from mmdetect.config import TransportConfig
_TRANSPORT_REGISTRY: dict[str, type[AbstractTransport]] = {
    "wifi": WifiTransport,
}
def create_transport(cfg: TransportConfig) -> AbstractTransport:
    cls = _TRANSPORT_REGISTRY.get(cfg.kind)
    if cls is None:
        raise ValueError(
            f"Unknown transport kind: {cfg.kind!r}. "
            f"Available: {list(_TRANSPORT_REGISTRY.keys())}"
        )
    if cfg.kind == "wifi":
        return cls(bind_addr=cfg.host, port=cfg.port)
    if cfg.kind == "serial":
        return cls(port=cfg.serial_port, baud_rate=cfg.baud_rate)
    return cls()
__all__ = [
    "AbstractTransport",
    "TransportState",
    "WifiTransport",
    "create_transport",
]