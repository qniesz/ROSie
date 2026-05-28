"""navigator.py — PointNavigator: drive robot to a clicked map coordinate.

Uses the fused SLAM+odom pose to execute a simple two-phase
rotate-then-drive straight-line navigation.  No obstacle avoidance —
the caller must ensure the path is clear.

Usage
-----
    nav = PointNavigator(serial, get_pose_fn, on_status_fn)
    nav.navigate_to(1.5, 2.0)                   # start moving
    nav.navigate_to(0.0, 0.0, on_arrived=save)  # new target replaces old
    nav.stop()                                  # abort + zero motors
"""

import logging
import math
import threading
import time
from typing import Callable, Optional, Tuple

from .commands import handle_cmd_vel

logger = logging.getLogger("rosie.navigator")

# ── Control parameters ─────────────────────────────────────────────────────
ANGULAR_SPEED = 0.40   # rad/s  — pure-rotation phase
LINEAR_SPEED  = 0.20   # m/s   — forward drive speed
ANGLE_TOL     = 0.10   # rad   — switch from rotating to driving (≈5.7°)
DIST_TOL      = 0.08   # m     — arrival threshold (8 cm)
STEER_GAIN    = 1.00   # proportional heading-correction gain while driving
CONTROL_HZ    = 10     # control-loop frequency


def _angle_diff(a: float, b: float) -> float:
    """Signed difference a − b, normalised to (−π, π]."""
    d = (a - b) % (2 * math.pi)
    if d > math.pi:
        d -= 2 * math.pi
    return d


class PointNavigator:
    """Navigate the robot to a single (x, y) world-frame point.

    Parameters
    ----------
    serial :
        NeatoSerial instance already in TestMode (LDS active).
    get_pose_fn :
        Callable() → (x, y, theta, ...) or None.
        Extra elements beyond index 2 are ignored.
    on_status_fn :
        Optional callback(status_str) fired on navigation events.
        Possible values: ``"rotating"``, ``"driving"``, ``"arrived"``,
        ``"idle"``, ``"error"``.
        Called from the background thread — must not block.
    """

    def __init__(
        self,
        serial,
        get_pose_fn: Callable,
        on_status_fn: Optional[Callable] = None,
    ) -> None:
        self._serial = serial
        self._get_pose = get_pose_fn
        self._on_status = on_status_fn
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()
        self._target: Optional[Tuple[float, float]] = None

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    @property
    def is_navigating(self) -> bool:
        """True while a navigation thread is alive."""
        with self._lock:
            return self._thread is not None and self._thread.is_alive()

    @property
    def current_target(self) -> Optional[Tuple[float, float]]:
        with self._lock:
            return self._target

    def navigate_to(
        self,
        x: float,
        y: float,
        on_arrived: Optional[Callable] = None,
    ) -> None:
        """Start driving to (x, y); cancels any current move immediately."""
        with self._lock:
            if self._thread and self._thread.is_alive():
                logger.info(
                    "Navigator: replacing current move with new target (%.3f, %.3f)",
                    x, y,
                )
                self._stop_event.set()
            self._target = (x, y)
            self._stop_event.clear()
            self._thread = threading.Thread(
                target=self._run,
                args=(x, y, on_arrived),
                name="navigator",
                daemon=True,
            )
            self._thread.start()

    def stop(self) -> None:
        """Abort navigation and zero motors.  Blocks until thread exits."""
        logger.info("Navigator: stop() called")
        self._stop_event.set()
        try:
            handle_cmd_vel(self._serial, 0.0, 0.0)
        except Exception:
            pass
        with self._lock:
            t = self._thread
        if t and t.is_alive():
            t.join(timeout=3.0)
        with self._lock:
            self._thread = None
            self._target = None

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _emit_status(self, status: str) -> None:
        logger.debug("Navigator: %s", status)
        if self._on_status:
            try:
                self._on_status(status)
            except Exception:
                logger.debug("on_status callback raised", exc_info=True)

    def _run(
        self,
        tx: float,
        ty: float,
        on_arrived: Optional[Callable],
    ) -> None:
        """Background thread: rotate to face target, then drive to it."""
        arrived = False
        try:
            self._navigate_loop(tx, ty)
            arrived = not self._stop_event.is_set()
        except Exception:
            logger.exception("Navigator: unexpected error in loop")
            self._emit_status("error")
        finally:
            try:
                handle_cmd_vel(self._serial, 0.0, 0.0)
            except Exception:
                pass

        if arrived:
            logger.info("Navigator: arrived at (%.3f, %.3f)", tx, ty)
            self._emit_status("arrived")
            if on_arrived:
                try:
                    on_arrived()
                except Exception:
                    logger.exception("Navigator: on_arrived callback error")
        else:
            self._emit_status("idle")

    def _navigate_loop(self, tx: float, ty: float) -> None:
        period = 1.0 / CONTROL_HZ
        rotating = True
        self._emit_status("rotating")

        while not self._stop_event.is_set():
            pose = None
            try:
                pose = self._get_pose()
            except Exception:
                logger.debug("get_pose error", exc_info=True)

            if pose is None:
                time.sleep(period)
                continue

            cx, cy, ctheta = pose[0], pose[1], pose[2]
            dx = tx - cx
            dy = ty - cy
            dist = math.hypot(dx, dy)

            if dist < DIST_TOL:
                handle_cmd_vel(self._serial, 0.0, 0.0)
                return  # arrived normally

            target_heading = math.atan2(dy, dx)
            angle_err = _angle_diff(target_heading, ctheta)

            if rotating and abs(angle_err) < ANGLE_TOL:
                rotating = False
                self._emit_status("driving")

            if rotating:
                az = ANGULAR_SPEED if angle_err > 0 else -ANGULAR_SPEED
                handle_cmd_vel(self._serial, 0.0, az)
            else:
                # Re-enter rotation phase if heading has drifted far off target
                if abs(angle_err) > ANGLE_TOL * 2.0:
                    rotating = True
                    self._emit_status("rotating")
                    continue
                az = max(-ANGULAR_SPEED, min(ANGULAR_SPEED, angle_err * STEER_GAIN))
                handle_cmd_vel(self._serial, LINEAR_SPEED, az)

            time.sleep(period)

        # stop_event set — caller's finally block zeros motors
