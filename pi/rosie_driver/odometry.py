"""
odometry.py — Wheel encoder odometry for the Neato D6.

Reads wheel encoder positions via GetMotors, computes differential-drive
odometry (x, y, theta, velocities), and publishes over MQTT.

Based on the odometry math from LoyVanBeek/neato_ros2 and jnugen/neato_robot.
"""

import logging
import math
import os
import time
from dataclasses import dataclass

from .serial_handler import NeatoSerial, BASE_WIDTH_MM

logger = logging.getLogger(__name__)

BASE_WIDTH_M = BASE_WIDTH_MM / 1000.0

# Stall detection: if both wheel loads (0–100 %) exceed this threshold for
# STALL_MIN_FRAMES consecutive polls, the robot is assumed to be physically
# jammed (e.g. LDS turret bumping a cart underside).  Encoder deltas are
# zeroed during the stall so position doesn't drift.  Tunable via env vars.
STALL_LOAD_THRESHOLD = int(os.environ.get("STALL_LOAD_THRESHOLD", "75"))
STALL_MIN_FRAMES     = int(os.environ.get("STALL_MIN_FRAMES",     "2"))

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

    # Wheel load (0-100 %) and RPM — published for slip / stall observability
    left_load: float = 0.0
    right_load: float = 0.0
    left_rpm: float = 0.0
    right_rpm: float = 0.0

    # Stall state — True when bilateral high load freezes encoder deltas
    stall_active: bool = False
    _stall_count: int = 0

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

    # Stall detection: bilateral high load means wheels are straining against
    # a physical obstruction (e.g. LDS turret jammed on cart underside).
    # Read loads early so we can gate encoder deltas before kinematics.
    _left_load  = motor_state.get("LeftWheel_Load",  0.0)
    _right_load = motor_state.get("RightWheel_Load", 0.0)
    if _left_load > STALL_LOAD_THRESHOLD and _right_load > STALL_LOAD_THRESHOLD:
        odom._stall_count += 1
    else:
        odom._stall_count = 0
    _stall = odom._stall_count >= STALL_MIN_FRAMES
    if _stall and not odom.stall_active:
        logger.warning(
            "stall detected: left_load=%.0f%% right_load=%.0f%% — "
            "zeroing encoder deltas", _left_load, _right_load
        )
    elif not _stall and odom.stall_active:
        logger.info("stall cleared")
    odom.stall_active = _stall

    # Compute deltas
    d_left = (left_mm - odom._prev_left_mm) / 1000.0   # mm -> m
    d_right = (right_mm - odom._prev_right_mm) / 1000.0
    dt = now - odom._prev_time

    if dt <= 0:
        return odom

    # During a stall the wheels are spinning without real displacement.
    # Zero the deltas so position doesn't drift; _prev_mm is still advanced
    # below so stall-period counts are consumed and won't appear as a jump
    # on recovery.
    if _stall:
        d_left = 0.0
        d_right = 0.0

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
    odom.left_load  = _left_load
    odom.right_load = _right_load
    odom.left_rpm   = motor_state.get("LeftWheel_RPM",   0.0)
    odom.right_rpm  = motor_state.get("RightWheel_RPM",  0.0)
    odom._prev_left_mm = left_mm
    odom._prev_right_mm = right_mm
    odom._prev_time = now

    return odom
