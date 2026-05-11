"""
bumper_sensors.py — Physical bump switch reader.

Board-agnostic: uses gpio_backend.py which auto-detects Raspberry Pi
(RPi.GPIO / BCM numbering) or Orange Pi (gpiod / chip+line numbering)
based on /proc/device-tree/model and the board gpio.json profile.

Behaviour on trigger (physical or virtual):
  - Publishes bumper state to MQTT (rosie/bumpers)
  - Calls the registered stop callback to halt the robot via serial

Virtual triggers are fired by no_go_guard when the robot approaches a
no-go line, using the same stop + publish path as a physical hit.

Disable with ROSIE_BUMPER_ENABLED=0 (skips GPIO setup; virtual triggers
still publish MQTT but do not call the stop callback).
"""

import logging
import os
import threading
import time
from typing import TYPE_CHECKING, Callable, Optional

if TYPE_CHECKING:
    from .mqtt_bridge import MQTTBridge

from .gpio_backend import BumperGpioBackend, NullGpioBackend, create_bumper_gpio
from .gpio_pins import BUMPER_CHANNELS

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------
POLL_HZ       = 20     # reads per second
DEBOUNCE_SECS = 0.03   # 30 ms debounce before confirming a press

# ---------------------------------------------------------------------------
# Module state
# ---------------------------------------------------------------------------
_thread: threading.Thread | None = None
_stop_event = threading.Event()
_backend: BumperGpioBackend | None = None
_gpio_ready = False
_mqtt: "MQTTBridge | None" = None
_stop_callback: Optional[Callable[[], None]] = None
_state_lock = threading.Lock()

# Current reported state (False = not triggered)
_state: dict[str, bool] = {ch: False for ch in BUMPER_CHANNELS}

# Debounce tracking: key → timestamp of first sustained LOW reading
_low_since: dict[str, float] = {}

# Per-channel virtual pulse timers
_virtual_timers: dict[str, threading.Timer | None] = {ch: None for ch in BUMPER_CHANNELS}

# Set True while test_pin() is driving a pin LOW — suppresses stop callback
_test_active: bool = False

# Pose provider: called on bump to snapshot robot position
_pose_provider: Optional[Callable[[], tuple]] = None

# Bump event callback: called on every new bump trigger
_bump_event_callback: Optional[Callable] = None


def _env_flag(name: str, default: bool = True) -> bool:
    value = os.getenv(name)
    if value is None:
        return default
    return value.strip().lower() not in {"0", "false", "no", "off", "disabled"}


_enabled = _env_flag("ROSIE_BUMPER_ENABLED", default=True)


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------
def _setup_gpio() -> bool:
    global _backend, _gpio_ready
    b = create_bumper_gpio()
    try:
        ok = b.setup()
    except Exception as exc:
        logger.error("[bumper_sensors] GPIO setup failed: %s", exc)
        return False
    _backend = b
    if not ok or isinstance(b, NullGpioBackend):
        logger.warning("[bumper_sensors] GPIO not available — %s", b.describe())
        return False
    _gpio_ready = True
    logger.info("[bumper_sensors] GPIO ready — %s", b.describe())
    return True


def _publish(new_state: dict[str, bool]) -> None:
    """Update internal state and publish to MQTT."""
    global _state
    _state = dict(new_state)
    if _mqtt is not None:
        _mqtt.publish_bumpers(
            left_front=new_state["front_left"],
            right_front=new_state["front_right"],
            left_side=new_state["side_left"],
            right_side=new_state["side_right"],
        )


def _release_virtual_key(key: str) -> None:
    """Release a single virtual channel when its pulse timer expires."""
    _release_pin(key)
    with _state_lock:
        timer = _virtual_timers.get(key)
        if timer is not None:
            _virtual_timers[key] = None
        if not _state.get(key, False):
            return
        new_state = dict(_state)
        new_state[key] = False
        changed = new_state != _state
    if changed:
        _publish(new_state)


def _poll_loop() -> None:
    interval = 1.0 / POLL_HZ

    while not _stop_event.is_set():
        now = time.monotonic()

        if _gpio_ready and _backend is not None:
            try:
                new_state = dict(_state)
                any_new_trigger = False

                for key in BUMPER_CHANNELS:
                    raw_low = _backend.read_low(key)

                    if raw_low:
                        if key not in _low_since:
                            _low_since[key] = now
                        if (now - _low_since[key]) >= DEBOUNCE_SECS and not _state[key]:
                            new_state[key] = True
                            any_new_trigger = True
                            logger.info("[bumper_sensors] %s triggered (%s)", key, _backend.pin_display(key))
                    else:
                        _low_since.pop(key, None)
                        if _state[key]:
                            new_state[key] = False
                            logger.info("[bumper_sensors] %s released (%s)", key, _backend.pin_display(key))

                if new_state != _state:
                    _publish(new_state)

                if any_new_trigger and _stop_callback is not None and not _test_active:
                    try:
                        _stop_callback()
                    except Exception as exc:
                        logger.error("[bumper_sensors] stop callback error: %s", exc)

            except Exception as exc:
                logger.error("[bumper_sensors] poll error: %s", exc)

        time.sleep(interval)

    logger.info("[bumper_sensors] poll thread stopped")


# ---------------------------------------------------------------------------
# Public API — virtual trigger (called by no_go_guard)
# ---------------------------------------------------------------------------
def virtual_active() -> bool:
    """Return True if a virtual trigger is still in its hold window."""
    with _state_lock:
        return any(_state.values())


def trigger_virtual(
    front_left: bool = False,
    front_right: bool = False,
    side_left: bool = False,
    side_right: bool = False,
    hold_secs: float = 0.0,
    stop_on_trigger: bool = True,
) -> None:
    """Activate virtual bumper hits from no-go line detection.

    Merges with any currently-active physical bumper state, publishes to
    MQTT, and fires the stop callback if any new trigger is set.
    """
    requested = {
        "front_left": front_left,
        "front_right": front_right,
        "side_left": side_left,
        "side_right": side_right,
    }
    timers_to_start: list[threading.Timer] = []
    pins_to_drive: list[str] = []

    with _state_lock:
        new_state = dict(_state)
        any_new = False

        for key, wanted in requested.items():
            if not wanted:
                continue

            if not _state[key]:
                any_new = True
            new_state[key] = True
            pins_to_drive.append(key)

            old_timer = _virtual_timers.get(key)
            if old_timer is not None:
                old_timer.cancel()
                _virtual_timers[key] = None

            if hold_secs > 0:
                t = threading.Timer(hold_secs, _release_virtual_key, args=(key,))
                t.daemon = True
                _virtual_timers[key] = t
                timers_to_start.append(t)

        changed = new_state != _state

    # Drive GPIO pins LOW so the Neato firmware sees a real bumper press.
    for key in pins_to_drive:
        _drive_pin_low(key)

    for t in timers_to_start:
        t.start()

    if changed:
        _publish(new_state)
    if any_new and stop_on_trigger and _stop_callback is not None:
        try:
            _stop_callback()
        except Exception as exc:
            logger.error("[bumper_sensors] stop callback error: %s", exc)


def release_virtual() -> None:
    """Clear virtual bumper states, returning all pins to INPUT/floating."""
    # Release every pin back to INPUT so the Neato sees no bumper pressed.
    for key in BUMPER_CHANNELS:
        _release_pin(key)

    with _state_lock:
        for key, timer in _virtual_timers.items():
            if timer is not None:
                timer.cancel()
                _virtual_timers[key] = None

        if not _gpio_ready or _backend is None:
            new_state = {k: False for k in _state}
        else:
            new_state = {key: _backend.read_low(key) for key in BUMPER_CHANNELS}
        changed = new_state != _state

    if changed:
        _publish(new_state)


def set_stop_callback(fn: Optional[Callable[[], None]]) -> None:
    """Register a callback that halts the robot when a bumper fires."""
    global _stop_callback
    _stop_callback = fn


def _drive_pin_low(key: str) -> None:
    """Switch a bumper pin to OUTPUT LOW to fake a switch closure to the Neato."""
    if _backend is None or not _gpio_ready:
        return
    try:
        _backend.drive_low(key)
        logger.info("[bumper_sensors] %s (%s) -> LOW (press)", key, _backend.pin_display(key))
    except Exception as exc:
        logger.error("[bumper_sensors] _drive_pin_low(%s) failed: %s", key, exc)


def _release_pin(key: str) -> None:
    """Switch a bumper pin back to INPUT/floating so the Neato sees the switch released."""
    if _backend is None or not _gpio_ready:
        return
    try:
        _backend.release(key)
        logger.info("[bumper_sensors] %s (%s) -> INPUT (release)", key, _backend.pin_display(key))
    except Exception as exc:
        logger.error("[bumper_sensors] _release_pin(%s) failed: %s", key, exc)


def test_pin(key: str, hold_secs: float = 5.0) -> None:
    """Drive a bumper pin LOW for *hold_secs*, then restore to INPUT/floating.

    Used by the HA "Bumper Test" buttons so the user can verify wiring
    with a multimeter or confirm the poll loop detects the transition.
    """
    if _backend is None or not _gpio_ready:
        logger.warning("[bumper_sensors] test_pin(%s) — GPIO not ready", key)
        return

    def _drive():
        global _test_active
        _test_active = True
        try:
            _backend.drive_low(key)
            logger.info("[bumper_sensors] test_pin(%s) %s → LOW", key, _backend.pin_display(key))
            time.sleep(hold_secs)
        finally:
            _test_active = False
            _backend.release(key)
            logger.info("[bumper_sensors] test_pin(%s) %s → INPUT/floating", key, _backend.pin_display(key))

    threading.Thread(target=_drive, name=f"test_pin_{key}", daemon=True).start()


def set_pose_provider(fn: Optional[Callable[[], tuple]]) -> None:
    """Register a function that returns the current robot pose tuple."""
    global _pose_provider
    _pose_provider = fn


def set_bump_event_callback(fn: Optional[Callable]) -> None:
    """Register a callback invoked on each new bump trigger.

    Signature: fn(side, is_front, virtual, pose)
    """
    global _bump_event_callback
    _bump_event_callback = fn


def _emit_bump_event(side: str, is_front: bool, virtual: bool) -> None:
    """Snapshot pose and fire the bump event callback if registered."""
    if _bump_event_callback is None:
        return
    pose = None
    if _pose_provider is not None:
        try:
            pose = _pose_provider()
        except Exception:
            pass
    try:
        _bump_event_callback(side, is_front, virtual, pose)
    except Exception as exc:
        logger.error("[bumper_sensors] bump event callback error: %s", exc)


# ---------------------------------------------------------------------------
# Lifecycle
# ---------------------------------------------------------------------------
def start(
    mqtt_bridge: "MQTTBridge | None" = None,
    stop_callback: Optional[Callable[[], None]] = None,
) -> None:
    """Set up GPIO and start the background polling thread."""
    global _thread, _mqtt, _stop_callback

    if not _enabled:
        logger.info("[bumper_sensors] disabled by ROSIE_BUMPER_ENABLED=0")
        return

    _mqtt = mqtt_bridge
    if stop_callback is not None:
        _stop_callback = stop_callback

    if not _setup_gpio():
        return

    # Poll loop disabled — pins are PUD_OFF for safe connection to the
    # vac board (output-only via test_pin).  Bumper state comes from serial.
    logger.info("[bumper_sensors] GPIO ready (output-only, no poll loop)")


def stop() -> None:
    """Stop the polling thread."""
    global _thread
    _stop_event.set()
    if _thread is not None:
        _thread.join(timeout=2.0)
        _thread = None
    logger.info("[bumper_sensors] stopped")
