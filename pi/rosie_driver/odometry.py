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

# Slip detection: wheels spinning freely (high RPM + low load = no traction).
# Opposite of stall — wheels turn fast because there is nothing to push against.
SLIP_RPM_THRESHOLD  = int(os.environ.get("SLIP_RPM_THRESHOLD",  "30"))
SLIP_LOAD_THRESHOLD = int(os.environ.get("SLIP_LOAD_THRESHOLD", "20"))
SLIP_MIN_FRAMES     = int(os.environ.get("SLIP_MIN_FRAMES",     "3"))

# Single-wheel stall: one wheel physically blocked (e.g. pressed against a
# table leg) while the other spins.  Bilateral threshold is not met in this
# case, so we use a higher load threshold + near-zero RPM on ONE wheel.
# Encoder deltas are zeroed for both wheels to avoid phantom pivot drift.
SINGLE_STALL_LOAD_THRESHOLD = int(os.environ.get("SINGLE_STALL_LOAD_THRESHOLD", "85"))
SINGLE_STALL_RPM_THRESHOLD  = int(os.environ.get("SINGLE_STALL_RPM_THRESHOLD",  "25"))

# Tilt suppression: if the robot is pitched/rolled beyond this angle the
# kinematic model is unreliable (climbing a rug edge, wedged on furniture).
TILT_THRESHOLD_DEG  = float(os.environ.get("TILT_THRESHOLD_DEG", "8.0"))

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

    # Stall state — True when bilateral high load freezes encoder deltas,
    # or when one wheel is physically blocked (high load + near-zero RPM).
    stall_active: bool = False
    _stall_count: int = 0
    _single_stall_count: int = 0

    # Slip state — True when high RPM + low load indicates free spinning
    slip_active: bool = False
    _slip_count: int = 0

    # Tilt state — True when accelerometer reports robot is pitched/rolled
    tilt_active: bool = False

    # Latest accelerometer readings (for MQTT publish)
    accel_pitch: float = 0.0
    accel_roll: float = 0.0
    accel_sum_g: float = 1.0

    # Previous encoder values
    _prev_left_mm: float = 0.0
    _prev_right_mm: float = 0.0
    _prev_time: float = 0.0
    _initialized: bool = False


@dataclass
class AccelState:
    """Accelerometer/tilt reading from GetAccel."""
    pitch_deg: float = 0.0
    roll_deg: float = 0.0
    sum_g: float = 1.0
    timestamp: float = 0.0


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


def get_accel(serial: NeatoSerial) -> "AccelState | None":
    """
    Read accelerometer / tilt data via GetAccel.

    Returns an AccelState, or None if the command fails or returns no data.
    """
    lines = serial.send_and_collect("GetAccel", "Label", timeout=1.0)
    parsed: dict[str, float] = {}
    for line in lines:
        parts = line.split(",")
        if len(parts) >= 2:
            try:
                parsed[parts[0].strip()] = float(parts[1].strip())
            except ValueError:
                continue
    if not parsed:
        return None
    return AccelState(
        pitch_deg=parsed.get("PitchInDegrees", 0.0),
        roll_deg=parsed.get("RollInDegrees", 0.0),
        sum_g=parsed.get("SumInG", 1.0),
        timestamp=time.monotonic(),
    )


def update_odometry(odom: OdomState, motor_state: dict[str, float],
                    accel: "AccelState | None" = None) -> OdomState:
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

    # Read loads and RPMs early — needed for both stall and slip detection.
    # NOTE: The Neato reports this field as "LeftWheel_Load%" (with the % sign).
    _left_load  = motor_state.get("LeftWheel_Load%",  0.0)
    _right_load = motor_state.get("RightWheel_Load%", 0.0)
    _left_rpm   = motor_state.get("LeftWheel_RPM",   0.0)
    _right_rpm  = motor_state.get("RightWheel_RPM",  0.0)

    # --- Stall detection (bilateral high load = physically jammed) ---
    if _left_load > STALL_LOAD_THRESHOLD and _right_load > STALL_LOAD_THRESHOLD:
        odom._stall_count += 1
    else:
        odom._stall_count = 0
    _stall = odom._stall_count >= STALL_MIN_FRAMES

    # --- Single-wheel block (one wheel against table leg, other spinning) ---
    # Higher load threshold + near-zero RPM → one wheel is physically stopped.
    _left_blocked  = (_left_load  > SINGLE_STALL_LOAD_THRESHOLD
                      and abs(_left_rpm)  < SINGLE_STALL_RPM_THRESHOLD)
    _right_blocked = (_right_load > SINGLE_STALL_LOAD_THRESHOLD
                      and abs(_right_rpm) < SINGLE_STALL_RPM_THRESHOLD)
    if (_left_blocked or _right_blocked) and not _stall:
        odom._single_stall_count += 1
    else:
        odom._single_stall_count = 0
    _single_stall = odom._single_stall_count >= STALL_MIN_FRAMES

    _any_stall = _stall or _single_stall
    if _any_stall and not odom.stall_active:
        if _single_stall:
            side = "left" if _left_blocked else "right"
            logger.warning(
                "single-wheel stall: %s blocked load=%.0f%% rpm=%.0f — "
                "zeroing encoder deltas",
                side,
                _left_load if _left_blocked else _right_load,
                _left_rpm  if _left_blocked else _right_rpm,
            )
        else:
            logger.warning(
                "stall detected: left_load=%.0f%% right_load=%.0f%% — "
                "zeroing encoder deltas", _left_load, _right_load
            )
    elif not _any_stall and odom.stall_active:
        logger.info("stall cleared")
    odom.stall_active = _any_stall

    # --- Slip detection (high RPM + low load = spinning freely, no traction) ---
    # Both wheels must show free-spin simultaneously: during a normal turn one
    # wheel is loaded while the other is light, so OR would false-positive.
    _left_slip  = abs(_left_rpm)  > SLIP_RPM_THRESHOLD and _left_load  < SLIP_LOAD_THRESHOLD
    _right_slip = abs(_right_rpm) > SLIP_RPM_THRESHOLD and _right_load < SLIP_LOAD_THRESHOLD
    if _left_slip and _right_slip:
        odom._slip_count += 1
    else:
        odom._slip_count = 0
    _slip = odom._slip_count >= SLIP_MIN_FRAMES
    if _slip and not odom.slip_active:
        logger.warning(
            "slip detected: left=%.0frpm/%.0f%% right=%.0frpm/%.0f%% — "
            "zeroing encoder deltas",
            _left_rpm, _left_load, _right_rpm, _right_load,
        )
    elif not _slip and odom.slip_active:
        logger.info("slip cleared")
    odom.slip_active = _slip

    # --- Tilt suppression (accelerometer: robot climbing rug or wedged) ---
    _tilt = False
    if accel is not None:
        _tilt = (
            abs(accel.pitch_deg) > TILT_THRESHOLD_DEG
            or abs(accel.roll_deg) > TILT_THRESHOLD_DEG
        )
        if _tilt and not odom.tilt_active:
            logger.warning(
                "tilt detected: pitch=%.1f\u00b0 roll=%.1f\u00b0 — "
                "zeroing encoder deltas",
                accel.pitch_deg, accel.roll_deg,
            )
        elif not _tilt and odom.tilt_active:
            logger.info("tilt cleared")
        odom.tilt_active = _tilt
        odom.accel_pitch = accel.pitch_deg
        odom.accel_roll  = accel.roll_deg
        odom.accel_sum_g = accel.sum_g
    else:
        odom.tilt_active = False

    # Compute deltas
    d_left = (left_mm - odom._prev_left_mm) / 1000.0   # mm -> m
    d_right = (right_mm - odom._prev_right_mm) / 1000.0
    dt = now - odom._prev_time

    if dt <= 0:
        return odom

    # Zero encoder deltas when wheels are spinning freely (no traction),
    # or when any stall condition is active (bilateral or single-wheel block).
    if _slip or _any_stall:
        d_left = 0.0
        d_right = 0.0

    # Zero encoder deltas when both wheels are reversing — bounce-back motion
    # creates phantom odometry that corrupts the map.
    if d_left < 0.0 and d_right < 0.0:
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
    odom.left_rpm   = _left_rpm
    odom.right_rpm  = _right_rpm
    odom._prev_left_mm = left_mm
    odom._prev_right_mm = right_mm
    odom._prev_time = now

    return odom
