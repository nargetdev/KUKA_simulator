"""Constants mirroring the values from the reference KRL programs."""

from __future__ import annotations

from dataclasses import dataclass

# Command identifiers ---------------------------------------------------------

CMD_MOTION = 0

CMD_TOOL_1 = 1
CMD_TOOL_2 = 2
CMD_TOOL_3 = 3
CMD_TOOL_4 = 4
CMD_TOOL_5 = 5
CMD_TOOL_6 = 6
CMD_TOOL_7 = 7
CMD_TOOL_8 = 8
CMD_TOOL_9 = 9

CMD_WCS_1 = 54
CMD_WCS_2 = 55
CMD_WCS_3 = 56
CMD_WCS_4 = 57
CMD_WCS_5 = 58
CMD_WCS_6 = 59

CMD_ROBOT_HOME_G28 = 28
CMD_WORKSPACE_HOME_G30 = 30

TOOL_COMMANDS = {
    CMD_TOOL_1: 1,
    CMD_TOOL_2: 2,
    CMD_TOOL_3: 3,
    CMD_TOOL_4: 4,
    CMD_TOOL_5: 5,
    CMD_TOOL_6: 6,
    CMD_TOOL_7: 7,
    CMD_TOOL_8: 8,
    CMD_TOOL_9: 9,
}

BASE_COMMANDS = {
    CMD_WCS_1: 1,
    CMD_WCS_2: 2,
    CMD_WCS_3: 3,
    CMD_WCS_4: 4,
    CMD_WCS_5: 5,
    CMD_WCS_6: 6,
}

# Bitmask constants -----------------------------------------------------------

BITMASK_X = 1 << 0
BITMASK_Y = 1 << 1
BITMASK_Z = 1 << 2
BITMASK_A = 1 << 3
BITMASK_B = 1 << 4
BITMASK_C = 1 << 5
BITMASK_F = 1 << 6
BITMASK_E = 1 << 7

# Motion related defaults -----------------------------------------------------

MAXIMUM_CP_SPEED_LIMIT_M_PER_S = 1.2  # Matches MAXIMUM_CP_SPEED_LIMIT in KRL
DEFAULT_TOOL = 1
DEFAULT_BASE = 5
DEFAULT_FEED_MM_PER_S = 1150.0

# Feedback buffers ------------------------------------------------------------
FEEDBACK_FILL_VALUE = -27


@dataclass(frozen=True)
class HomePose:
    """Fallback home pose (matches T5_B2_HOME in the references)."""

    x: float = 0.0
    y: float = 0.0
    z: float = 500.0
    a: float = 0.0
    b: float = 0.0
    c: float = 0.0

