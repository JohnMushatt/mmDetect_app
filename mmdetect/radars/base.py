from future import annotations
from abc import ABC, abstractmethod
from dataclasses import dataclass, field


@dataclass(slots=True, frozen=True)
class Detection:
    """
    Single instance of radar target detection
    """
    x_local : float
    y_local : float
    speed_mps : float
    timestamp_s : float
    radar_id : str = ""
@dataClass(slots=True)
class DetectionFrame:
    """ 
    One frame of detections from a single radar
    """
    radar_id : str
    timestamp_s : float
    detections: tuple[Detection, ...] = field(default_factory=tuple)

class AbstractRadarDriver(ABC):
    """
    Interface for each radar type
    """

    @abstractmethod
    def parse(self, data: btes) DetectionFrame | None:
        """
        Parse raw bytes into a DetectionFrame object.
        """
        ...
    @property
    @abstractmethod
    def max_targets(self) -> int:
        """
        Returns the maximum number of targets that can be detected by the radar.
        """
        ...
    @property
    @abstractmethod
    def max_speed(self) -> float:
        """
        Returns the maximum speed that can be detected by the radar.
        """
        ...


