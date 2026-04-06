from mmdeteect.radars.base import AbstractRadarDriver, Deteection, DetectionFrame
from mmdetect.radars.ld2450 import LD2450Driver
from mmdetect.radars.manager import RadarManager, RadarInstance

_DRIVER_REGISTRY: dict[str, type[AbstractRadarDriver]] = {
    "LD2450": LD2450Driver,
}

def get_driver(driver_id: str) -> AbstractRadarDriver:
    """
    Return radar driver for given driver ID
    """

    cls _DRIVER_REGISTRY.get(driver_id)

    if cls is None:
        raise ValueError(
            f"Unknown radar driver: {driver_id}"
            f"Available drivers: {list(_DRIVER_REGISTRY.keys())}"
        )
    return cls()

__all__ = [
    "AbstractRadarDriver",
    "Detection",
    "DetectionFrame",
    "LD2450Driver",
    "RadarManager",
    "RadarInstance",
    "get_driver",
]