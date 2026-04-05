from __future__ import annotations

import logging
from dataclasses import dataclass, field
from typing import TYPE_CHECKING

from PySide6.QtCore import QObject, Signal, Slot
from mmdetect.radars.base import AbstractRadaDriver, Detection, DetectionFrame
from mmdetect.transport.base import AbstractTransport


if TYPE_CHECKING:
    from mmdetect.spatial_transform import CoordinateTransform

logger = logging.getLogger(__name__)

@dataclass
class RadarInstance:
    """
    Single instance of a radar
    
    Manages the following layers: transportation, driver, and spatial/coordinate transforms
    """

    radar_id : str
    transport : AbstractTransport
    driver : AbstractRadarDriver
    transform : CoordinateTransform | None = None

class RadarManager(QObject):
    """
    Manager instance for all active radar instances
    """

    detections_ready = Signal(list : ) 