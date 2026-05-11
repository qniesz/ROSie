"""no_go_guard.py - No-go line enforcement via bumper sensor injection.

When the robot approaches a configured no-go line, the geometry engine
determines which side (left / right) and direction (front / side) the line
is on, and fires _touch_callback(side, is_front, touching=True) so that the
corresponding virtual bump switch is activated.  This stops the robot and
publishes the bumper state to HA via MQTT.

The robot is modelled as a 33 cm × 33 cm square with the flat edge at the
front.  When any edge of that square touches a no-go line the callback fires.

dot_fwd >= dot_rgt → approach from front/rear → front bumper
dot_rgt >  dot_fwd → approach from side       → side bumper
"""

import json
import logging
import math
import os
import threading
import time
from typing import Any, Callable, Optional

logger = logging.getLogger(__name__)


def _env_flag(name: str, default: bool = True) -> bool:
    value = os.getenv(name)
    if value is None:
        return default
    return value.strip().lower() not in {"0", "false", "no", "off", "disabled"}

# Default no-go lines in MAP frame (metres, SLAM-corrected coordinates)
DEFAULT_LINES: list[tuple[tuple[float, float], tuple[float, float]]] = []

# ---------------------------------------------------------------------------
# Tuning
# ---------------------------------------------------------------------------
# The Neato D6 is modelled as a square (~33 cm × 33 cm) with the flat
# edge as the front.  When ANY edge of that square touches a no-go line,
# the HAL channel on that side gets a voltage tap.
HALF_LENGTH = 0.165   # metres — center to front/back edge
HALF_WIDTH  = 0.165   # metres — center to left/right edge

# How often to log robot position during cleaning (seconds, 0 = disable)
POS_LOG_INTERVAL = 5.0

# ---------------------------------------------------------------------------
# Module state
# ---------------------------------------------------------------------------
# Per-channel touch state: 'on' = currently touching a no-go line.
# 'is_front' = True if the touch approach was front/rear, False if side.
_channels: dict[str, dict] = {
    "left":  {"on": False, "is_front": False},
    "right": {"on": False, "is_front": False},
}

# Called when a channel touch state changes: fn(side, is_front, touching)
_touch_callback: Optional[Callable[[str, bool, bool], None]] = None

_last_pos_log   = 0.0
_lock           = threading.Lock()
_enabled        = _env_flag("ROSIE_NOGO_ENABLED", default=True)
_lines          = list(DEFAULT_LINES)
_trigger_callback = None   # fn(x, y, line_idx, dist) — called on first TAP-ON


def set_footprint(
    half_length: float | None = None,
    half_width: float | None = None,
) -> None:
    """Update the robot footprint used for no-go line proximity checks."""
    global HALF_LENGTH, HALF_WIDTH
    if half_length is not None:
        HALF_LENGTH = float(half_length)
    if half_width is not None:
        HALF_WIDTH = float(half_width)
    logger.debug("[no_go_guard] footprint updated: half_length=%.3f half_width=%.3f", HALF_LENGTH, HALF_WIDTH)


def set_touch_callback(fn: Optional[Callable[[str, bool, bool], None]]) -> None:
    """Register a callback invoked when a channel's touch state changes.

    Signature: fn(side: str, is_front: bool, touching: bool)
      side     — 'left' or 'right'
      is_front — True if line is approached from front/rear, False if from side
      touching — True on first contact, False on release
    """
    global _touch_callback
    _touch_callback = fn


# ---------------------------------------------------------------------------
# Geometry
# ---------------------------------------------------------------------------
def _line_distance(x: float, y: float, p1: tuple[float, float],
                   p2: tuple[float, float]) -> float:
    """Perpendicular distance from point (x, y) to an INFINITE line."""
    x1, y1 = p1
    x2, y2 = p2
    dx = x2 - x1
    dy = y2 - y1
    length = math.hypot(dx, dy)
    if length == 0.0:
        return math.hypot(x - x1, y - y1)
    return abs(dy * (x - x1) - dx * (y - y1)) / length


def _foot_vector(x: float, y: float, p1: tuple[float, float],
                 p2: tuple[float, float]) -> tuple[float, float]:
    """Vector from (x, y) to the foot of the perpendicular on line p1-p2."""
    x1, y1 = p1
    x2, y2 = p2
    dx = x2 - x1
    dy = y2 - y1
    length_sq = dx * dx + dy * dy
    if length_sq > 1e-12:
        t = ((x - x1) * dx + (y - y1) * dy) / length_sq
        return (x1 + t * dx - x, y1 + t * dy - y)
    return (x1 - x, y1 - y)


def _normalize_point(raw_point: Any, point_name: str) -> tuple[float, float]:
    if not isinstance(raw_point, (list, tuple)) or len(raw_point) != 2:
        raise ValueError(f"{point_name} must be [x, y]")
    return (float(raw_point[0]), float(raw_point[1]))


def _normalize_lines(raw_lines: Any) -> list[tuple[tuple[float, float],
                                                    tuple[float, float]]]:
    if not isinstance(raw_lines, (list, tuple)):
        raise ValueError("lines must be a list")

    normalized = []
    for idx, raw_line in enumerate(raw_lines, start=1):
        if isinstance(raw_line, dict):
            p1_raw = raw_line.get("p1")
            p2_raw = raw_line.get("p2")
        elif isinstance(raw_line, (list, tuple)) and len(raw_line) == 2:
            p1_raw, p2_raw = raw_line
        else:
            raise ValueError(f"line {idx} must contain p1/p2")

        p1 = _normalize_point(p1_raw, f"line {idx} p1")
        p2 = _normalize_point(p2_raw, f"line {idx} p2")
        if p1 == p2:
            raise ValueError(f"line {idx} has identical p1 and p2")
        normalized.append((p1, p2))

    return normalized


def export_lines() -> list[dict[str, list[float]]]:
    """Return active no-go lines in a JSON-friendly form."""
    with _lock:
        lines_snapshot = list(_lines)

    return [
        {
            "p1": [round(p1[0], 4), round(p1[1], 4)],
            "p2": [round(p2[0], 4), round(p2[1], 4)],
        }
        for p1, p2 in lines_snapshot
    ]


def line_count() -> int:
    with _lock:
        return len(_lines)


def is_enabled() -> bool:
    return _enabled


def set_trigger_callback(fn) -> None:
    """Register a callback invoked when the robot first crosses a no-go line.

    Signature: fn(x: float, y: float, line_idx: int, dist: float)
    Pass None to clear.
    """
    global _trigger_callback
    _trigger_callback = fn


def set_lines(raw_lines: Any) -> tuple[bool, str, list[dict[str, list[float]]]]:
    """Validate and replace active no-go lines at runtime."""
    global _lines

    try:
        normalized = _normalize_lines(raw_lines)
    except Exception as exc:
        return False, f"invalid no-go lines: {exc}", export_lines()

    releases: list[tuple[str, bool]] = []
    with _lock:
        _lines = normalized
        if not _lines:
            for ch_name, ch in _channels.items():
                if ch["on"]:
                    releases.append((ch_name, ch["is_front"]))
                    ch["on"] = False

    for ch_name, is_front in releases:
        if _touch_callback is not None:
            _touch_callback(ch_name, is_front, False)

    return True, f"loaded {len(normalized)} no-go line(s)", export_lines()


def load_lines_file(path: str) -> tuple[bool, str]:
    """Load no-go lines from JSON file if present."""
    if not path:
        return False, "no path provided"

    try:
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
    except FileNotFoundError:
        return False, "file not found; using built-in defaults"
    except Exception as exc:
        return False, f"failed to read lines file: {exc}"

    lines_raw = data.get("lines") if isinstance(data, dict) else data
    ok, msg, _ = set_lines(lines_raw)
    if ok:
        return True, f"{msg} from {path}"
    return False, msg


def save_lines_file(path: str) -> tuple[bool, str]:
    """Persist active no-go lines to JSON file."""
    if not path:
        return False, "no path provided"

    try:
        parent = os.path.dirname(path)
        if parent:
            os.makedirs(parent, exist_ok=True)

        payload = {
            "lines": export_lines(),
            "updated_at": int(time.time()),
        }
        tmp_path = f"{path}.tmp"
        with open(tmp_path, "w", encoding="utf-8") as f:
            json.dump(payload, f, indent=2)
        os.replace(tmp_path, path)
        return True, f"saved {len(payload['lines'])} line(s) to {path}"
    except Exception as exc:
        return False, f"failed to save lines file: {exc}"


# ---------------------------------------------------------------------------
# Side detection
# ---------------------------------------------------------------------------
def _side_channels(theta: float, vec_x: float, vec_y: float) -> list[str]:
    """Return which channel ('left' or 'right') a line affects.

    Based on the cross product of the robot heading and the vector toward
    the nearest point on the line:
        cross > 0  → line is to the robot's LEFT  → left channel
        cross ≤ 0  → line is to the robot's RIGHT → right channel

    Always returns exactly one channel — never both.
    """
    hx = math.cos(theta)
    hy = math.sin(theta)
    cross = hx * vec_y - hy * vec_x
    return ["left"] if cross > 0 else ["right"]


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------
def init():
    """No-op — no GPIO setup required (bumper output handled by bumper_sensors.py)."""
    if not _enabled:
        logger.info("[no_go_guard] disabled by ROSIE_NOGO_ENABLED")
    else:
        logger.info("[no_go_guard] ready — %d line(s) active", line_count())


def check(x: float, y: float, theta: float = 0.0, angular_vel: float = 0.0,
          linear_vel: float = 0.0, pose_source: str = ""):
    """Evaluate robot position against all no-go lines.

    For each line the perpendicular distance from the robot centre is compared
    against the robot's effective reach using the rectangle support function:

        reach = HALF_LENGTH * dot_fwd + HALF_WIDTH * dot_rgt

    where dot_fwd / dot_rgt are the projections of the unit approach vector
    onto the robot's forward and right axes respectively.

    On first contact _touch_callback fires with (side, is_front=True/False,
    touching=True); on release it fires again with touching=False.

    dot_fwd >= dot_rgt → line approached from front/rear → is_front=True
    dot_rgt >  dot_fwd → line approached from side       → is_front=False

    If the robot is reversing (linear_vel < -0.01 m/s) all active touches are
    released immediately.
    """
    global _last_pos_log

    if not _enabled:
        return

    _callbacks: list[tuple[str, bool, bool]] = []  # (side, is_front, touching)
    _fired_trigger = None

    with _lock:
        now = time.monotonic()

        # Robot is reversing — release all active touches
        if linear_vel < -0.01:
            for ch_name, ch in _channels.items():
                if ch["on"]:
                    _callbacks.append((ch_name, ch["is_front"], False))
                    ch["on"] = False
                    logger.info(
                        "[no_go_guard] RELEASE (reversing %.3f m/s) %s",
                        linear_vel, ch_name.upper(),
                    )

        elif not _lines:
            for ch_name, ch in _channels.items():
                if ch["on"]:
                    _callbacks.append((ch_name, ch["is_front"], False))
                    ch["on"] = False

        else:
            # Robot forward and right unit vectors
            fwd_x =  math.cos(theta)
            fwd_y =  math.sin(theta)
            rgt_x =  fwd_y
            rgt_y = -fwd_x

            ch_touching: dict[str, bool] = {"left": False, "right": False}
            ch_is_front: dict[str, bool] = {"left": False, "right": False}
            ch_nearest:  dict[str, tuple[int, float]] = {
                "left":  (0, float("inf")),
                "right": (0, float("inf")),
            }

            for idx, (p1, p2) in enumerate(_lines, start=1):
                dist  = _line_distance(x, y, p1, p2)
                fvec  = _foot_vector(x, y, p1, p2)
                sides = _side_channels(theta, fvec[0], fvec[1])

                fvec_len = math.hypot(fvec[0], fvec[1])
                if fvec_len < 1e-9:
                    reach    = max(HALF_LENGTH, HALF_WIDTH)
                    is_front = True
                else:
                    n_x = fvec[0] / fvec_len
                    n_y = fvec[1] / fvec_len
                    # Signed projection onto robot forward axis.
                    # >0 = line is in front of robot center, <0 = behind.
                    dot_fwd_signed = n_x * fwd_x + n_y * fwd_y
                    # Only the FRONT half of the robot should trigger no-go.
                    # If the contact would be behind the center, skip this line.
                    if dot_fwd_signed < -0.05:
                        continue
                    dot_fwd = abs(dot_fwd_signed)
                    dot_rgt = abs(n_x * rgt_x + n_y * rgt_y)
                    reach    = HALF_LENGTH * dot_fwd + HALF_WIDTH * dot_rgt
                    is_front = dot_fwd >= dot_rgt

                for side in sides:
                    if dist < ch_nearest[side][1]:
                        ch_nearest[side] = (idx, dist)
                    if dist <= reach:
                        ch_touching[side] = True
                        ch_is_front[side] = is_front  # last touching line wins

            # Periodic position log
            if POS_LOG_INTERVAL > 0 and now - _last_pos_log >= POS_LOG_INTERVAL:
                _last_pos_log = now
                l_idx, l_dist = ch_nearest["left"]
                r_idx, r_dist = ch_nearest["right"]
                logger.info(
                    "[no_go_guard] pos=(%.3f,%.3f) hdg=%.1f°  "
                    "L=line%d/%.3fm/%s  R=line%d/%.3fm/%s",
                    x, y, math.degrees(theta),
                    l_idx, l_dist, "TOUCH" if ch_touching["left"] else "---",
                    r_idx, r_dist, "TOUCH" if ch_touching["right"] else "---",
                )

            for ch_name, ch in _channels.items():
                touching = ch_touching[ch_name]
                if touching and not ch["on"]:
                    ch["on"] = True
                    ch["is_front"] = ch_is_front[ch_name]
                    line_idx, dist = ch_nearest[ch_name]
                    logger.info(
                        "[no_go_guard] TOUCH (%s/%s) pos=(%.3f,%.3f) hdg=%.1f° "
                        "line=%d dist=%.3fm",
                        ch_name.upper(),
                        "FRONT" if ch["is_front"] else "SIDE",
                        x, y, math.degrees(theta), line_idx, dist,
                    )
                    _callbacks.append((ch_name, ch["is_front"], True))
                    if _fired_trigger is None:
                        _fired_trigger = (x, y, line_idx, dist)
                elif not touching and ch["on"]:
                    _callbacks.append((ch_name, ch["is_front"], False))
                    ch["on"] = False

    # Fire all callbacks outside the lock
    for cb_args in _callbacks:
        if _touch_callback is not None:
            try:
                _touch_callback(*cb_args)
            except Exception as exc:
                logger.error("[no_go_guard] touch callback error: %s", exc)

    if _fired_trigger is not None and _trigger_callback is not None:
        try:
            _trigger_callback(*_fired_trigger)
        except Exception as exc:
            logger.error("[no_go_guard] trigger callback error: %s", exc)


def cleanup():
    """No-op — no GPIO resources to release."""
    pass
