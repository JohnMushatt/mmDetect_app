from __future__ import annotations
from pathlib import Path
from pydantic import BaseModel, Field
import yaml

class PoseConfig(BaseModel):
    x_m: float = 0.0
    y_m: float = 0.0
    heading_deg: float = 0.0

class TransportConfig(BaseModel):
    kind: str = "wifi" # WiFi or Serial
    host: str = "0.0.0.0"
    port: int Field(default=5000, ge=1024, le=65535)

    serial_port: str = ""
    baud_rate: int = 256000
class RadarConfig(BaseModel):
    id: str
    driver: str = "ld2450"
    transport: TransportConfig = TransportConfig()
    pose: PoseConfig = PoseConfig()

class RoomConfig(BaseModel):
    name: str = "Default Room"
    width_m: float = Field(default=6.0, gt=0)
    length_m: float = Field(default=4.0, gt=0)

    radars: list[RadarConfig] = []

class TrackingConfig(BaseModel):
    max_association_dist: float = Field(default=0.5, gt=0)
    max_stale_frames: int = Field(default=20, ge=1)
    trail_length: int = Field(default=100, ge=1)


class AppConfig(BaseModel):
    rooms: list[RoomConfig] = []
    tracking: TrackingConfig = TrackingConfig()
def load_config(path: Path | str) -> AppConfig:
    path = Path(path)

    if not path.exists():

        return AppConfig()

    with open(path) as f:
        raw = yaml.safe_load(f) or {}
    return AppConfig.model_validate(raw)

def save_config(config: AppConfig, path: Path | str) -> None:

    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w") as f:
        yaml.dump(config.model_dump(), f, default_flow_style=False, sort_keys=False)
        