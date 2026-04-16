from __future__ import annotations

import struct
from dataclasses import dataclass, field
from typing import List
from enum import IntEnum

FRAME_HEADER = b'\xBB\xBB'
FRAME_TAIL = b'\xEE\xEE'

PROTO_OVERHEAD = 8  # header(2) + msg_type(1) + payload_len(2) + checksum(1) + tail(2)
MAX_TARGETS = 3
TARGET_STRUCT = struct.Struct("<hhhH")  # x(i16) y(i16) speed(i16) resolution(u16)


class MsgType(IntEnum):
    TARGET = 0x01
    IQ = 0x02
@dataclass(slots=True, frozen=True)
class Target:
    x_mm: int
    y_mm: int
    speed_cms: int
    resolution_mm: int

    @property
    def distance_mm(self) -> float:
        return (self.x_mm **2 + self.y_mm **2) ** 0.5

    @property
    def angle_deg(self) -> float:
        import math
        return math.degrees(math.atan2(self.x_mm, self.y_mm))

    @property
    def is_valid(self) -> bool:
        return not (self.x_mm == 0 and self.y_mm == 0 and self.speed_cms == 0 and self.resolution_mm == 0)
@dataclass
class TargetFrame:
        frame_id: int
        timestamp_ms: int
        targets: tuple[Target, ...] = field(default_factory=tuple)

        @property
        def target_count(self) -> int:
            return len(self.targets)
        @staticmethod
        def _verify_envelope(data: bytes) -> tuple[int, int]:
            """Validate header/tail/checksum and return (msg_type, payload_len)."""
            if len(data) < PROTO_OVERHEAD:
                raise ValueError(f"Packet too short ({len(data)} bytes)")
            if data[:2] != FRAME_HEADER:
                raise ValueError("Invalid header")
            if data[-2:] != FRAME_TAIL:
                raise ValueError("Invalid tail")
            msg_type = data[2]
            payload_len = struct.unpack_from("<H", data, 3)[0]
            expected_size = PROTO_OVERHEAD + payload_len
            if len(data) != expected_size:
                raise ValueError(
                    f"Length mismatch: payload_len says {expected_size} bytes, got {len(data)}"
                )
            cs_end = 5 + payload_len  # index of checksum byte
            expected_cs = data[cs_end]
            computed_cs = 0
            for b in data[2:cs_end]:
                computed_cs ^= b
            if computed_cs != expected_cs:
                raise ValueError(
                    f"Checksum mismatch: expected 0x{expected_cs:02X}, got 0x{computed_cs:02X}"
                )
            return msg_type, payload_len
        @classmethod
        def from_bytes(cls, data: bytes) -> TargetFrame:
            msg_type, payload_len = cls._verify_envelope(data)

            if msg_type != MsgType.TARGET:
                raise ValueError(f"Expected TARGET message type (0x01), got 0x{msg_type:02X}")
            frame_id, timestamp_ms = struct.unpack_from("<II", data, 5)

            target_count: int = data[13]

            targets: List[Target] = []
            offset = 14
            for i in range(MAX_TARGETS):
                x, y, spd, res = TARGET_STRUCT.unpack_from(data, offset)
                offset += TARGET_STRUCT.size
                if i < target_count:
                    targets.append(Target(x, y, spd, res))
               # else:
                #
            return cls(frame_id, timestamp_ms, tuple(targets))

        @classmethod
        def compute_checksum(cls, data: bytes) -> int:
            checksum = 0
            for byte in data:
                checksum ^= byte
            return checksum

        def to_bytes(self) -> bytes:
            payload = struct.pack("<IIB", self.frame_id, self.timestamp_ms, self.target_count)
            for target in range(MAX_TARGETS):
                if target < self.target_count:
                    t = self.targets[target]
                    payload += TARGET_STRUCT.pack(t.x_mm, t.y_mm, t.speed_cms, t.resolution_mm)
                else:
                    payload += TARGET_STRUCT.pack(0, 0, 0, 0)

            checksum = self.compute_checksum(payload)
            payload += bytes([checksum])
            return FRAME_HEADER + payload + FRAME_TAIL

            