from __future__ import annotations
import math
from dataclasses import dataclass
from mmdetect.radars.base import Detection


@dataclass(frozen=True, slots=True)
class RadarPose:
    x_m : float         # radar's position in the room coordinate system
    y_m : float         # radar's position in the room coordinate system
    heading_deg : float # radar's forward direction relative to the room coordinate system

class CoordinateTransform:
    def __init__(self, pose: RadarPose) -> None:
        self._pose = pose
        theta = math.radians(pose.heading_deg)
        self._cos = math.cos(theta)
        self._sin = math.sin(theta)

    @property
    def pose(self) -> RadarPose:
        return self._pose

    def to_room(self, det: Detection) -> Detection:
        """
        Converts local (radar local) coordinate to room coordinates
        """
       #x_room_local = self._pose.x_m + det.x_local * self._cos + det.y_local * self._sin
        #y_room_local = self._pose.y_m + det.x_local * self._sin - det.y_local * self._cos
        x_room_local = self._pose.x_m - det.y_local * self._cos - det.x_local * self._sin
        y_room_local = self._pose.y_m - det.y_local * self._sin - det.x_local * self._cos
        #x_room_local = self._pose.x_m + det.x_local * self._sin - det.y_local * self._cos
        #y_room_local = self._pose.y_m + det.x_local * self._cos + det.y_local * self._sin
        return Detection(
            x_local=x_room_local,
            y_local=y_room_local,
            speed_mps=det.speed_mps,
            timestamp_s=det.timestamp_s,
            radar_id=det.radar_id
        )

    def to_local(self, det: Detection) -> Detection:
        """
        Converts room coordinate to radar local coordinates
        """
        dx = det.x_local - self._pose.x_m
        dy = det.y_local - self._pose.y_m
        x_local = dx * self._cos + dy * self._sin
        y_local = -dx * self._sin + dy * self._cos

        det : Detection = Detection(
            x_local=x_local,
            y_local=y_local,
            speed_mps=det.speed_mps,
            timestamp_s=det.timestamp_s,
            radar_id=det.radar_id
        )
        return det

    