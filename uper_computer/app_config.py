from __future__ import annotations

import math
import os
from dataclasses import dataclass

from robot_protocol import VelocityCommand, build_velocity_frame


COMMON_BAUDRATES = [9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600]
DEFAULT_BAUDRATE = 115200
SEND_PERIOD_MS = 100
STOP_REPEAT_COUNT = 3
MIN_LINEAR_SPEED_M_S = 0.05
MAX_LINEAR_SPEED_M_S = 5.0
DEFAULT_LINEAR_SPEED_M_S = 0.05
LINEAR_SPEED_STEP_M_S = 0.05
MIN_ANGULAR_SPEED_RAD_S = 0.01
MAX_ANGULAR_SPEED_RAD_S = 10.0
DEFAULT_ANGULAR_SPEED_RAD_S = 0.01
ANGULAR_SPEED_STEP_RAD_S = 0.01
MAX_LOG_LINES = 500

CONFIG_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "robot_config.json")


@dataclass(frozen=True)
class MotionState:
    forward: bool = False
    backward: bool = False
    left: bool = False
    right: bool = False
    counterclockwise: bool = False
    clockwise: bool = False


def calculate_velocity(
    state: MotionState,
    linear_speed_m_s: float,
    angular_speed_rad_s: float,
) -> VelocityCommand:
    vx_axis = int(state.right) - int(state.left)
    vy_axis = int(state.forward) - int(state.backward)
    wz_axis = int(state.clockwise) - int(state.counterclockwise)
    linear_speed_mm_s = linear_speed_m_s * 1000.0

    if vx_axis != 0 and vy_axis != 0:
        linear_speed_mm_s /= math.sqrt(2.0)

    return VelocityCommand(
        vx_mm_s=round(vx_axis * linear_speed_mm_s),
        vy_mm_s=round(vy_axis * linear_speed_mm_s),
        wz_mrad_s=round(wz_axis * angular_speed_rad_s * 1000.0),
    )


ZERO_FRAME = build_velocity_frame(VelocityCommand())
