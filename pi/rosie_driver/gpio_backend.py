"""GPIO backends for ROSie bumper/no-go injection."""

from __future__ import annotations

import logging
import os
from typing import Any

from .gpio_pins import BUMPER_CHANNELS, BumperPin, GpioProfile, load_gpio_profile

logger = logging.getLogger(__name__)


class BumperGpioBackend:
    name = "base"
    available = False

    def __init__(self, profile: GpioProfile):
        self.profile = profile

    def setup(self) -> bool:
        return False

    def read_low(self, key: str) -> bool:
        return False

    def drive_low(self, key: str) -> None:
        raise RuntimeError("GPIO backend is not available")

    def release(self, key: str) -> None:
        return None

    def cleanup(self) -> None:
        return None

    def pin_display(self, key: str) -> str:
        pin = self.profile.pins.get(key)
        return pin.display if pin else key

    def describe(self) -> str:
        return f"{self.name} profile={self.profile.name} status={self.profile.status}"


class NullGpioBackend(BumperGpioBackend):
    name = "null"
    available = False

    def __init__(self, profile: GpioProfile, reason: str):
        super().__init__(profile)
        self.reason = reason

    def setup(self) -> bool:
        logger.warning("[gpio_backend] GPIO disabled: %s", self.reason)
        return False

    def drive_low(self, key: str) -> None:
        logger.warning("[gpio_backend] ignoring %s drive_low: %s", key, self.reason)

    def describe(self) -> str:
        return f"null profile={self.profile.name} reason={self.reason}"


class RPiGpioBackend(BumperGpioBackend):
    name = "rpi_gpio"
    available = True

    def __init__(self, profile: GpioProfile):
        super().__init__(profile)
        try:
            import RPi.GPIO as gpio  # type: ignore[import-not-found]
        except ImportError as exc:
            raise RuntimeError("RPi.GPIO is not installed") from exc
        self._gpio = gpio
        self._validate_pins("bcm")

    def setup(self) -> bool:
        self._gpio.setmode(self._gpio.BCM)
        self._gpio.setwarnings(False)
        for key in BUMPER_CHANNELS:
            self.release(key)
        return True

    def read_low(self, key: str) -> bool:
        pin = self._pin(key)
        return self._gpio.input(pin.bcm) == self._gpio.LOW

    def drive_low(self, key: str) -> None:
        pin = self._pin(key)
        self._gpio.setup(pin.bcm, self._gpio.OUT)
        self._gpio.output(pin.bcm, self._gpio.LOW)

    def release(self, key: str) -> None:
        pin = self._pin(key)
        self._gpio.setup(pin.bcm, self._gpio.IN, pull_up_down=self._gpio.PUD_OFF)

    def cleanup(self) -> None:
        try:
            self._gpio.cleanup()
        except Exception:
            logger.debug("[gpio_backend] RPi.GPIO cleanup failed", exc_info=True)

    def _pin(self, key: str) -> BumperPin:
        pin = self.profile.pins[key]
        if pin.bcm is None:
            raise RuntimeError(f"{key} has no BCM pin in profile {self.profile.name}")
        return pin

    def _validate_pins(self, attr: str) -> None:
        missing = [key for key in BUMPER_CHANNELS if getattr(self.profile.pins.get(key), attr, None) is None]
        if missing:
            raise RuntimeError(f"profile {self.profile.name} missing {attr} pins: {', '.join(missing)}")


class GpiodBackend(BumperGpioBackend):
    name = "gpiod"
    available = True

    def __init__(self, profile: GpioProfile):
        super().__init__(profile)
        try:
            import gpiod  # type: ignore[import-not-found]
        except ImportError as exc:
            raise RuntimeError("python3-libgpiod is not installed") from exc
        self._gpiod = gpiod
        self._requests: dict[str, Any] = {}
        self._request_modes: dict[str, str] = {}
        self._api = "v2" if hasattr(gpiod, "request_lines") else "v1"
        self._validate_pins()

    def setup(self) -> bool:
        for key in BUMPER_CHANNELS:
            self.release(key)
        return True

    def read_low(self, key: str) -> bool:
        self._ensure_input(key)
        request = self._requests[key]
        pin = self._pin(key)
        if self._api == "v2":
            return _value_is_low(request.get_value(pin.line))
        return request.get_value() == 0

    def drive_low(self, key: str) -> None:
        self._release_request(key)
        pin = self._pin(key)
        if self._api == "v2":
            self._requests[key] = self._request_v2(pin, "output")
        else:
            self._requests[key] = self._request_v1(pin, "output")
        self._request_modes[key] = "output"

    def release(self, key: str) -> None:
        self._release_request(key)
        pin = self._pin(key)
        if self._api == "v2":
            self._requests[key] = self._request_v2(pin, "input")
        else:
            self._requests[key] = self._request_v1(pin, "input")
        self._request_modes[key] = "input"

    def cleanup(self) -> None:
        for key in list(self._requests):
            self._release_request(key)

    def _ensure_input(self, key: str) -> None:
        if self._request_modes.get(key) != "input":
            self.release(key)

    def _request_v2(self, pin: BumperPin, direction: str) -> Any:
        line_mod = getattr(self._gpiod, "line")
        direction_value = line_mod.Direction.OUTPUT if direction == "output" else line_mod.Direction.INPUT
        settings_kwargs: dict[str, Any] = {"direction": direction_value}
        if direction == "output":
            settings_kwargs["output_value"] = line_mod.Value.INACTIVE
        else:
            # Explicitly disable hardware pull-up/pull-down so the pin is truly
            # floating.  Without this, Bias.AS_IS may leave an H618 pull-down
            # active from a previous gpioset session, which would hold the line
            # LOW and make the Neato think a bumper is permanently pressed.
            settings_kwargs["bias"] = line_mod.Bias.DISABLED
        settings = self._gpiod.LineSettings(**settings_kwargs)
        return self._gpiod.request_lines(
            pin.chip,
            consumer="rosie",
            config={pin.line: settings},
        )

    def _request_v1(self, pin: BumperPin, direction: str) -> Any:
        chip = self._gpiod.Chip(pin.chip)
        line = chip.get_line(pin.line)
        if direction == "output":
            line.request(consumer="rosie", type=self._gpiod.LINE_REQ_DIR_OUT, default_vals=[0])
        else:
            line.request(consumer="rosie", type=self._gpiod.LINE_REQ_DIR_IN)
        return _GpiodV1Request(chip, line)

    def _release_request(self, key: str) -> None:
        request = self._requests.pop(key, None)
        self._request_modes.pop(key, None)
        if request is None:
            return
        try:
            request.release()
        except Exception:
            logger.debug("[gpio_backend] release failed for %s", key, exc_info=True)

    def _pin(self, key: str) -> BumperPin:
        pin = self.profile.pins[key]
        if pin.chip is None or pin.line is None:
            raise RuntimeError(f"{key} has no gpiod chip/line in profile {self.profile.name}")
        return pin

    def _validate_pins(self) -> None:
        missing = [
            key for key in BUMPER_CHANNELS
            if key not in self.profile.pins
            or self.profile.pins[key].chip is None
            or self.profile.pins[key].line is None
        ]
        if missing:
            raise RuntimeError(
                f"profile {self.profile.name} missing gpiod chip/line pins: {', '.join(missing)}"
            )


class _GpiodV1Request:
    def __init__(self, chip: Any, line: Any):
        self._chip = chip
        self._line = line

    def get_value(self) -> int:
        return int(self._line.get_value())

    def release(self) -> None:
        try:
            self._line.release()
        finally:
            self._chip.close()


def create_bumper_gpio(profile_name: str | None = None) -> BumperGpioBackend:
    profile = load_gpio_profile(profile_name)
    requested_backend = (os.getenv("ROSIE_GPIO_BACKEND") or profile.backend or "auto").strip().lower()
    if requested_backend == "auto":
        requested_backend = profile.backend

    try:
        if requested_backend == "rpi_gpio":
            return RPiGpioBackend(profile)
        if requested_backend == "gpiod":
            return GpiodBackend(profile)
        if requested_backend in {"none", "null", "disabled"}:
            return NullGpioBackend(profile, f"backend set to {requested_backend}")
        return NullGpioBackend(profile, f"unsupported backend {requested_backend!r}")
    except Exception as exc:
        return NullGpioBackend(profile, str(exc))


def _value_is_low(value: Any) -> bool:
    if isinstance(value, int):
        return value == 0
    name = getattr(value, "name", "").upper()
    if name:
        return name in {"INACTIVE", "LOW"}
    try:
        return int(value) == 0
    except Exception:
        return False
