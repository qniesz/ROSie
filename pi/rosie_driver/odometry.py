"""
odometry.py — Wheel encoder odometry for the Neato D6.

Reads wheel encoder positions via GetMotors, computes differential-drive
odometry (x, y, theta, velocities), and publishes over MQTT.

Based on the odometry math from LoyVanBeek/neato_ros2 and jnugen/neato_robot.
"""

import logging
import math
import time
from dataclasses import dataclass

from .serial_handler import NeatoSerial, BASE_WIDTH_MM

logger = logging.getLogger(__name__)

BASE_WIDTH_M = BASE_WIDTH_MM / 1000.0

# Motor status fields returned by GetMotors (in order)
MOTOR_FIELDS = [
    "Brush_RPM", "Brush_mA",
    "Vacuum_RPM", "Vacuum_mA",
    "LeftWheel_RPM", "LeftWheel_Load", "LeftWheel_PositionInMM", "LeftWheel_Speed",
    "RightWheel_RPM", "RightWheel_Load", "RightWheel_PositionInMM", "RightWheel_Speed",
    "SideBrush_mA",
]


@dataclass
class OdomState:
    """Accumulated odometry state."""
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0
    linear_vel: float = 0.0
    angular_vel: float = 0.0
    timestamp: float = 0.0

    # Previous encoder values
    _prev_left_mm: float = 0.0
    _prev_right_mm: float = 0.0
    _prev_time: float = 0.0
    _initialized: bool = False


def get_motors(serial: NeatoSerial) -> dict[str, float]:
    """
    Read motor/encoder state from the Neato.

    Returns dict with keys like 'LeftWheel_PositionInMM', 'RightWheel_PositionInMM', etc.
    """
    lines = serial.send_and_collect("GetMotors", "Parameter", timeout=1.0)
    state = {}
    for line in lines:
        parts = line.split(",")
        if len(parts) >= 2:
            try:
                state[parts[0].strip()] = float(parts[1].strip())
            except ValueError:
                continue
    return state


def update_odometry(odom: OdomState, motor_state: dict[str, float]) -> OdomState:
    """
    Update odometry from new motor encoder readings.

    Uses differential drive kinematics:
        d_center = (d_left + d_right) / 2
        d_theta  = (d_right - d_left) / base_width
    """
    left_mm = motor_state.get("LeftWheel_PositionInMM", 0.0)
    right_mm = motor_state.get("RightWheel_PositionInMM", 0.0)
    now = time.monotonic()

    if not odom._initialized:
        odom._prev_left_mm = left_mm
        odom._prev_right_mm = right_mm
        odom._prev_time = now
        odom._initialized = True
        odom.timestamp = now
        return odom

    # Compute deltas
    d_left = (left_mm - odom._prev_left_mm) / 1000.0   # mm -> m
    d_right = (right_mm - odom._prev_right_mm) / 1000.0
    dt = now - odom._prev_time

    if dt <= 0:
        return odom

    # Differential drive kinematics
    d_center = (d_left + d_right) / 2.0
    d_theta = (d_right - d_left) / BASE_WIDTH_M

    # Mid-angle approximation for pose update
    mid_theta = odom.theta + d_theta / 2.0
    odom.x += d_center * math.cos(mid_theta)
    odom.y += d_center * math.sin(mid_theta)
    odom.theta += d_theta

    # Normalize theta to [-pi, pi]
    odom.theta = math.atan2(math.sin(odom.theta), math.cos(odom.theta))

    # Velocities
    odom.linear_vel = d_center / dt
    odom.angular_vel = d_theta / dt
    odom.timestamp = now

    # Store for next iteration
    odom._prev_left_mm = left_mm
    odom._prev_right_mm = right_mm
    odom._prev_time = now

    return odom
