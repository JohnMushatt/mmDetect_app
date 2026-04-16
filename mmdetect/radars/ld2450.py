from __future__ import annotations

import logging

from mmdetect.models.target import TargetFrame, MAX_TARGETS
from mmdetect.radars.base import AbstractRadarDriver, DetectionFrame, Detection

logger = logging.getLogger(__name__)

MM_TO_M = 1e-3
MS_TO_S = 1e-3
CMS_TO_MPS = 1e-2

class LD2450Driver(AbstractRadarDriver):
    """
    Driver for the LD2450 radar
    
    Consymes LD2450 specific protocol over serial/UDP
    """

    @property
    def max_targets(self) -> int:
        return MAX_TARGETS
    @property
    def max_speed(self) -> float:
        return 10.0 # m/s
    def parse(self, data: bytes) -> DetectionFrame | None:
        # Try to call TargetFrame.from_bytes to parse incoming byte stream
        try:
            frame = TargetFrame.from_bytes(data)
        except ValueError as err:
            logger.debug("Invalid target frame: %s", err)
            return None

        # Extract timestamp and convert to seconds
        timestamp_s = frame.timestamp_ms * MS_TO_S

        detections = tuple(

            Detection(
                x_local= t.x_mm * MM_TO_M,
                y_local= t.y_mm * MM_TO_M,
                speed_mps= t.speed_cms * CMS_TO_MPS,
                timestamp_s = timestamp_s,
            )
            for t in frame.targets
            if t.is_valid
        )

        return DetectionFrame(
            radar_id="",
            timestamp_s = timestamp_s,
            detections = detections,
        )
    
        