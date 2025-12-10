"""Runtime state tracking for the virtual robot."""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from typing import Dict, Tuple

from .constants import DEFAULT_BASE, DEFAULT_FEED_MM_PER_S, DEFAULT_TOOL, HomePose


@dataclass
class Pose:
    x: float
    y: float
    z: float
    a: float
    b: float
    c: float

    def interpolate(self, other: "Pose", ratio: float) -> "Pose":
        return Pose(
            x=self.x + (other.x - self.x) * ratio,
            y=self.y + (other.y - self.y) * ratio,
            z=self.z + (other.z - self.z) * ratio,
            a=self.a + (other.a - self.a) * ratio,
            b=self.b + (other.b - self.b) * ratio,
            c=self.c + (other.c - self.c) * ratio,
        )


@dataclass(frozen=True)
class StateSnapshot:
    pose: Pose
    axes: Tuple[float, float, float, float, float, float]
    velocity_m_s: float
    tool: int
    base: int
    g_seq: int
    seq_in_flight: int
    queue_size: int
    feed_mm_per_s: float
    extrusion_rate: float
    flags: Dict[int, bool]


@dataclass
class RobotState:
    pose: Pose = field(
        default_factory=lambda: (
            lambda home=HomePose(): Pose(home.x, home.y, home.z, home.a, home.b, home.c)
        )()
    )
    axes: Tuple[float, float, float, float, float, float] = field(
        default_factory=lambda: (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    )
    velocity_m_s: float = 0.0
    tool: int = DEFAULT_TOOL
    base: int = DEFAULT_BASE
    g_seq: int = -1
    seq_in_flight: int = -1
    queue_size: int = 0
    feed_mm_per_s: float = DEFAULT_FEED_MM_PER_S
    extrusion_rate: float = 0.0
    flags: Dict[int, bool] = field(default_factory=lambda: {1: False, 2: False, 3: False, 4: False})

    def snapshot(self) -> StateSnapshot:
        return StateSnapshot(
            pose=replace(self.pose),
            axes=tuple(self.axes),
            velocity_m_s=self.velocity_m_s,
            tool=self.tool,
            base=self.base,
            g_seq=self.g_seq,
            seq_in_flight=self.seq_in_flight,
            queue_size=self.queue_size,
            feed_mm_per_s=self.feed_mm_per_s,
            extrusion_rate=self.extrusion_rate,
            flags=dict(self.flags),
        )

    def set_flag(self, index: int, value: bool) -> None:
        self.flags[index] = value

    def set_pose(self, pose: Pose) -> None:
        self.pose = pose
        self.axes = (pose.a, pose.b, pose.c, 0.0, 0.0, 0.0)

    def set_velocity(self, velocity_m_s: float) -> None:
        self.velocity_m_s = velocity_m_s

    def set_feed(self, feed_mm_per_s: float) -> None:
        self.feed_mm_per_s = feed_mm_per_s

    def set_queue_size(self, size: int) -> None:
        self.queue_size = size


