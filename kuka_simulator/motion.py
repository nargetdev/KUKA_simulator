"""Motion execution logic for the virtual robot."""

from __future__ import annotations

import asyncio
import math

from . import constants
from .protocol import GCodeCommand
from .state import Pose, RobotState


class MotionController:
    """Consumes decoded gcode commands and updates the robot state."""

    def __init__(
        self,
        state: RobotState,
        *,
        tick_interval: float = 0.02,
        max_speed_m_s: float = constants.MAXIMUM_CP_SPEED_LIMIT_M_PER_S,
    ) -> None:
        self._state = state
        self._tick_interval = tick_interval
        self._max_speed_m_s = max_speed_m_s

    async def execute(self, command: GCodeCommand) -> None:
        if command.cmd_type == constants.CMD_MOTION:
            await self._execute_motion(command)
            return

        if command.cmd_type in constants.TOOL_COMMANDS:
            self._state.tool = constants.TOOL_COMMANDS[command.cmd_type]
            return

        if command.cmd_type in constants.BASE_COMMANDS:
            self._state.base = constants.BASE_COMMANDS[command.cmd_type]
            return

        if command.cmd_type in (constants.CMD_ROBOT_HOME_G28, constants.CMD_WORKSPACE_HOME_G30):
            home = constants.HomePose()
            self._state.set_pose(Pose(home.x, home.y, home.z, home.a, home.b, home.c))
            return
        # Unknown commands are ignored for now.

    async def _execute_motion(self, command: GCodeCommand) -> None:
        target = self._apply_bitmask(command)
        feed_mm_per_s = self._select_feed(command)
        speed_m_s = min(feed_mm_per_s / 1000.0, self._max_speed_m_s)
        self._state.set_feed(feed_mm_per_s)
        self._state.set_velocity(speed_m_s)

        duration = self._compute_duration(self._state.pose, target, feed_mm_per_s)
        steps = max(1, int(duration / self._tick_interval)) if duration > 0 else 1
        start_pose = self._state.pose

        for step in range(1, steps + 1):
            ratio = step / steps
            pose = start_pose.interpolate(target, ratio)
            self._state.set_pose(pose)
            await asyncio.sleep(self._tick_interval)

        self._state.set_pose(target)
        self._state.set_velocity(0.0)
        if command.bitmask & constants.BITMASK_E:
            self._state.extrusion_rate = command.e

    def _apply_bitmask(self, command: GCodeCommand) -> Pose:
        pose = self._state.pose
        return Pose(
            x=command.x if command.bitmask & constants.BITMASK_X else pose.x,
            y=command.y if command.bitmask & constants.BITMASK_Y else pose.y,
            z=command.z if command.bitmask & constants.BITMASK_Z else pose.z,
            a=command.a if command.bitmask & constants.BITMASK_A else pose.a,
            b=command.b if command.bitmask & constants.BITMASK_B else pose.b,
            c=command.c if command.bitmask & constants.BITMASK_C else pose.c,
        )

    def _select_feed(self, command: GCodeCommand) -> float:
        if command.bitmask & constants.BITMASK_F and command.f > 0:
            return command.f
        return max(self._state.feed_mm_per_s, 1.0)

    @staticmethod
    def _compute_duration(pose: Pose, target: Pose, feed_mm_per_s: float) -> float:
        if feed_mm_per_s <= 0:
            return 0.0
        dx = target.x - pose.x
        dy = target.y - pose.y
        dz = target.z - pose.z
        distance_mm = math.sqrt(dx * dx + dy * dy + dz * dz)
        return distance_mm / feed_mm_per_s

