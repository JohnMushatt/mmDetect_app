from __future__ import annotations
from dataclasses import dataclass, field
from mmdetect.spatial.transform import RadarPose

@dataclass
class Room:
    name : str
    width_m : float
    length_m : float
    radar_poses : list[RadarPose] = field(Default_factory=list)
    