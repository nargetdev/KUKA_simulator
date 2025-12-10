"""Binary protocol helpers for the simulated EKI interfaces."""

from __future__ import annotations

import struct
from dataclasses import dataclass

from .constants import FEEDBACK_FILL_VALUE
from .state import StateSnapshot

GCODE_STRUCT = struct.Struct("<ii8fi")
FEEDBACK_STRUCT = struct.Struct("<12i")


@dataclass(slots=True)
class GCodeCommand:
    seq: int
    cmd_type: int
    x: float
    y: float
    z: float
    a: float
    b: float
    c: float
    f: float
    e: float
    bitmask: int

    @classmethod
    def from_bytes(cls, payload: bytes) -> "GCodeCommand":
        if len(payload) < 48:
            raise ValueError(f"Expected 48 bytes but received {len(payload)}")
        # Only first 44 bytes have meaningful data; remaining bytes are padding.
        unpacked = GCODE_STRUCT.unpack_from(payload)
        return cls(*unpacked)

    def to_bytes(self) -> bytes:
        body = GCODE_STRUCT.pack(
            self.seq,
            self.cmd_type,
            self.x,
            self.y,
            self.z,
            self.a,
            self.b,
            self.c,
            self.f,
            self.e,
            self.bitmask,
        )
        return body + b"\x00" * (48 - len(body))


def pack_feedback(seq: int, queue_size: int) -> bytes:
    """Encode the synchronous queue feedback buffer."""

    filler_count = FEEDBACK_STRUCT.size // 4 - 2
    filler = [FEEDBACK_FILL_VALUE] * filler_count
    return FEEDBACK_STRUCT.pack(seq, queue_size, *filler)


def pack_telemetry(snapshot: StateSnapshot) -> bytes:
    """Pack telemetry buffer replicating `_2_telemetry.sub`."""

    flags_word = 0
    for idx, value in snapshot.flags.items():
        if value:
            flags_word |= 1 << (idx - 1)

    buffer = bytearray()

    buffer += struct.pack(
        "<6f",
        snapshot.pose.x,
        snapshot.pose.y,
        snapshot.pose.z,
        snapshot.pose.a,
        snapshot.pose.b,
        snapshot.pose.c,
    )

    buffer += struct.pack("<6f", *snapshot.axes)

    buffer += struct.pack(
        "<4i",
        snapshot.g_seq,
        snapshot.seq_in_flight,
        snapshot.queue_size,
        flags_word,
    )

    buffer += struct.pack(
        "<f7i",
        snapshot.velocity_m_s,
        snapshot.tool,
        snapshot.base,
        76,
        80,
        84,
        88,
        92,
    )

    buffer += struct.pack("<8i", 96, 100, 104, 108, 112, 116, 120, 124)

    return bytes(buffer)

