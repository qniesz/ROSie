"""
bumper_sensors.py — Physical bump switch reader.

Four digital bump switches (normally open, pulled to GND when triggered):

    GPIO5  (physical pin 29) — Front Left
    GPIO6  (physical pin 31) — Front Right
    GPIO13 (physical pin 33) — Side Left
    GPIO26 (physical pin 37) — Side Right

Each switch connects between the GPIO pin and GND.
The Pi's internal pull-up keeps the pin HIGH when idle.
When the switch closes (bumped), the pin is pulled LOW.

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

try:
    import RPi.GPIO as GPIO  # type: ignore[import-not-found]
except (ImportError, RuntimeError):
    GPIO = None

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------
PIN_FRONT_LEFT  = 5    # GPIO5  — front left bump
PIN_FRONT_RIGHT = 6    # GPIO6  — front right bump
PIN_SIDE_LEFT   = 13   # GPIO13 — side left bump
PIN_SIDE_RIGHT  = 26   # GPIO26 — side right bump

POLL_HZ       = 20     # reads per second
DEBOUNCE_SECS = 0.03   # 30 ms debounce before confirming a press

_PINS = [
    (PIN_FRONT_LEFT,  "front_left"),
    (PIN_FRONT_RIGHT, "front_right"),
    (PIN_SIDE_LEFT,   "side_left"),
    (PIN_SIDE_RIGHT,  "side_right"),
]

# ---------------------------------------------------------------------------
# Module state
# ---------------------------------------------------------------------------
_thread: threading.Thread | None = None
_stop_event = threading.Event()
_gpio_ready = False
_mqtt: "MQTTBridge | None" = None
_stop_callback: Optional[Callable[[], None]] = None
_state_lock = threading.Lock()

# Current reported state (False = not triggered)
_state: dict[str, bool] = {
    "front_left":  False,
    "front_right": False,
    "side_left":   False,
    "side_right":  False,
}

# Debounce tracking: pin → timestamp of first sustained LOW reading
_low_since: dict[int, float] = {}

# Per-channel virtual pulse timers
_virtual_timers: dict[str, threading.Timer | None] = {
    "front_left": None,
    "front_right": None,
    "side_left": None,
    "side_right": None,
}

# Set True while test_pin() is driving a pin LOW — suppresses stop callback
_test_active: bool = False


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
    global _gpio_ready
    if GPIO is None:
        logger.warning("[bumper_sensors] RPi.GPIO not available — physical bumpers disabled")
        return False
    try:
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        for pin, _ in _PINS:
            GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_OFF)
        _gpio_ready = True
        logger.info(
            "[bumper_sensors] GPIO ready — FL=GPIO%d FR=GPIO%d SL=GPIO%d SR=GPIO%d (floating)",
            PIN_FRONT_LEFT, PIN_FRONT_RIGHT, PIN_SIDE_LEFT, PIN_SIDE_RIGHT,
        )
        return True
    except Exception as exc:
        logger.error("[bumper_sensors] GPIO setup failed: %s", exc)
        return False


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

        if _gpio_ready and GPIO is not None:
            try:
                new_state = dict(_state)
                any_new_trigger = False

                for pin, key in _PINS:
                    raw_low = GPIO.input(pin) == GPIO.LOW

                    if raw_low:
                        if pin not in _low_since:
                            _low_since[pin] = now
                        if (now - _low_since[pin]) >= DEBOUNCE_SECS and not _state[key]:
                            new_state[key] = True
                            any_new_trigger = True
                            logger.info("[bumper_sensors] %s triggered (GPIO%d)", key, pin)
                    else:
                        _low_since.pop(pin, None)
                        if _state[key]:
                            new_state[key] = False
                            logger.info("[bumper_sensors] %s released (GPIO%d)", key, pin)

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
    for key in _PIN_BY_KEY:
        _release_pin(key)

    with _state_lock:
        for key, timer in _virtual_timers.items():
            if timer is not None:
                timer.cancel()
                _virtual_timers[key] = None

        if not _gpio_ready or GPIO is None:
            new_state = {k: False for k in _state}
        else:
            new_state = {
                "front_left":  GPIO.input(PIN_FRONT_LEFT)  == GPIO.LOW,
                "front_right": GPIO.input(PIN_FRONT_RIGHT) == GPIO.LOW,
                "side_left":   GPIO.input(PIN_SIDE_LEFT)   == GPIO.LOW,
                "side_right":  GPIO.input(PIN_SIDE_RIGHT)  == GPIO.LOW,
            }
        changed = new_state != _state

    if changed:
        _publish(new_state)


def set_stop_callback(fn: Optional[Callable[[], None]]) -> None:
    """Register a callback that halts the robot when a bumper fires."""
    global _stop_callback
    _stop_callback = fn


_PIN_BY_KEY = {
    "front_left":  PIN_FRONT_LEFT,
    "front_right": PIN_FRONT_RIGHT,
    "side_left":   PIN_SIDE_LEFT,
    "side_right":  PIN_SIDE_RIGHT,
}


def _drive_pin_low(key: str) -> None:
    """Switch a bumper pin to OUTPUT LOW to fake a switch closure to the Neato."""
    pin = _PIN_BY_KEY.get(key)
    if pin is None or GPIO is None or not _gpio_ready:
        return
    try:
        GPIO.setup(pin, GPIO.OUT)
        GPIO.output(pin, GPIO.LOW)
        logger.info("[bumper_sensors] PIN %s (GPIO%d) -> LOW (press)", key, pin)
    except Exception as exc:
        logger.error("[bumper_sensors] _drive_pin_low(%s) failed: %s", key, exc)


def _release_pin(key: str) -> None:
    """Switch a bumper pin back to INPUT/floating so the Neato sees the switch released."""
    pin = _PIN_BY_KEY.get(key)
    if pin is None or GPIO is None or not _gpio_ready:
        return
    try:
        GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_OFF)
        logger.info("[bumper_sensors] PIN %s (GPIO%d) -> INPUT (release)", key, pin)
    except Exception as exc:
        logger.error("[bumper_sensors] _release_pin(%s) failed: %s", key, exc)


def test_pin(key: str, hold_secs: float = 5.0) -> None:
    """Drive a bumper pin LOW for *hold_secs*, then restore to INPUT w/ PUD_UP.

    Used by the HA "Bumper Test" buttons so the user can verify wiring
    with a multimeter or confirm the poll loop detects the transition.
    """
    pin = _PIN_BY_KEY.get(key)
    if pin is None or GPIO is None or not _gpio_ready:
        logger.warning("[bumper_sensors] test_pin(%s) — GPIO not ready", key)
        return

    def _drive():
        global _test_active
        _test_active = True
        try:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.LOW)
            logger.info("[bumper_sensors] test_pin(%s) GPIO%d → LOW", key, pin)
            time.sleep(hold_secs)
        finally:
            _test_active = False
            GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_OFF)
            logger.info("[bumper_sensors] test_pin(%s) GPIO%d → INPUT/floating", key, pin)

    threading.Thread(target=_drive, name=f"test_pin_{key}", daemon=True).start()


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
