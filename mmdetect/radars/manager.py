from __future__ import annotations

import logging
from dataclasses import dataclass, field
from typing import TYPE_CHECKING

from PySide6.QtCore import QObject, Signal, Slot
from mmdetect.radars.base import AbstractRadarDriver, Detection, DetectionFrame
from mmdetect.transport.base import AbstractTransport


if TYPE_CHECKING:
    from mmdetect.spatial.transform import CoordinateTransform

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

    detections_ready = Signal(list)
    raw_data_received = Signal(str, bytes)

    def __init__(self, parent: QObject | None = None) -> None:
        super().__init__(parent)
        self._radars : dict[str, RadarInstance] = {}


    @property
    def radar_ids(self) -> list[str]:
        return list(self._radars.keys())
    def add_radar(self, instance: RadarInstance) -> None:
        if instance.radar_id in self._radars:
            raise ValueError(f"Radar with ID {instance.radar_id} already exists")


        instance.transport.bytes_received.connect(
            self._make_handler(instance)
        )

        self._radars[instance.radar_id] = instance
        logger.info(f"Registered radar: {instance.radar_id}")

    def remove_radar(self, radar_id: str) -> None:
        if radar_id not in self._radars:
            raise ValueError(f"Radar with ID {radar_id} not found in self._radars")
        
        instance = self._radars.pop(radar_id, None)
        if instance is None:
            return
        instance.transport.stop()
        logger.info(f"Removed radar: {radar_id}")

    def start_all(self) -> None:
        for instance in self._radars.values():
            instance.transport.open()
    def stop_all(self) -> None:
        for instance in self._radars.values():
            instance.transport.close()
    def _make_handler(self, instance: RadarInstance):
        """
        Returns a slot bound to specified radar instance
        """
        @Slot(bytes)
        def _on_bytes(data: bytes) -> None:
            self.raw_data_received.emit(instance.radar_id, data)
            # Grab data from driver layer, should have a parse function that returns a DetectionFrame
            frame = instance.driver.parse(data)
            # Check if empty frame, return early if so
            if frame is None:
                logger.debug(f"Empty frame received from {instance.radar_id}")
                return
            detections = list(frame.detections)


            detections = [
                Detection(
                    x_local=d.x_local,
                    y_local=d.y_local,
                    speed_mps=d.speed_mps,
                    timestamp_s=d.timestamp_s,
                    radar_id=instance.radar_id,
                )
                for d in detections
            ]

            # Use coordinate tranform if defined 
            if instance.transform is not None:
                detections = [instance.transform.to_room(d) for d in detections]

            if detections:
                self.detections_ready.emit(detections)
        return _on_bytes
