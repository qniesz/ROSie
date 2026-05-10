"""Board-aware bumper GPIO pin profiles."""

from __future__ import annotations

import json
import os
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any

BUMPER_CHANNELS = ("front_left", "front_right", "side_left", "side_right")

_BOARD_ALIASES = {
    "auto": "auto",
    "rpi": "raspberrypi-zero2w",
    "raspberry": "raspberrypi-zero2w",
    "raspberrypi": "raspberrypi-zero2w",
    "raspberry-pi": "raspberrypi-zero2w",
    "raspberrypi-zero2w": "raspberrypi-zero2w",
    "raspberry-pi-zero-2-w": "raspberrypi-zero2w",
    "orangepi": "orangepi-zero2w",
    "orange-pi": "orangepi-zero2w",
    "orangepi-zero2w": "orangepi-zero2w",
    "orange-pi-zero-2-w": "orangepi-zero2w",
}


@dataclass(frozen=True)
class BumperPin:
    key: str
    physical_pin: int
    label: str
    bcm: int | None = None
    chip: str | None = None
    line: int | None = None

    @property
    def display(self) -> str:
        if self.bcm is not None:
            return f"{self.label} / physical pin {self.physical_pin}"
        if self.chip and self.line is not None:
            return f"{self.chip}:{self.line} / physical pin {self.physical_pin}"
        if self.label == f"physical pin {self.physical_pin}":
            return self.label
        return f"{self.label} / physical pin {self.physical_pin}"


@dataclass(frozen=True)
class GpioProfile:
    name: str
    backend: str
    status: str
    pins: dict[str, BumperPin]
    path: Path | None = None


def canonical_profile_name(value: str | None) -> str:
    raw = (value or "auto").strip().lower().replace("_", "-")
    return _BOARD_ALIASES.get(raw, raw)


def detect_board_profile() -> str:
    override = os.getenv("ROSIE_GPIO_PIN_PROFILE") or os.getenv("ROSIE_BOARD")
    profile = canonical_profile_name(override)
    if profile != "auto":
        return profile

    model = _read_text(Path("/proc/device-tree/model")).lower()
    if "raspberry pi" in model:
        return "raspberrypi-zero2w"
    if "orange pi" in model or "orangepi" in model:
        return "orangepi-zero2w"
    return "unknown"


def load_gpio_profile(profile_name: str | None = None) -> GpioProfile:
    name = canonical_profile_name(profile_name or detect_board_profile())
    custom_path = os.getenv("ROSIE_GPIO_PROFILE_PATH")
    if custom_path:
        path = Path(custom_path).expanduser()
    else:
        path = Path(__file__).resolve().parents[1] / "boards" / name / "gpio.json"

    if not path.exists():
        return GpioProfile(name=name, backend="null", status="missing", pins={}, path=path)

    data = json.loads(path.read_text(encoding="utf-8"))
    pins = {
        key: _pin_from_json(key, value)
        for key, value in data.get("pins", {}).items()
        if key in BUMPER_CHANNELS
    }
    pins = _apply_env_overrides(pins)
    return GpioProfile(
        name=data.get("board", name),
        backend=str(data.get("backend", "null")).strip().lower(),
        status=str(data.get("status", "unknown")),
        pins=pins,
        path=path,
    )


def _pin_from_json(key: str, value: dict[str, Any]) -> BumperPin:
    return BumperPin(
        key=key,
        physical_pin=int(value.get("physical_pin", 0)),
        label=str(value.get("label") or key),
        bcm=_optional_int(value.get("bcm")),
        chip=_optional_str(value.get("chip")),
        line=_optional_int(value.get("line")),
    )


def _apply_env_overrides(pins: dict[str, BumperPin]) -> dict[str, BumperPin]:
    result = dict(pins)
    for key in BUMPER_CHANNELS:
        env_name = f"ROSIE_GPIO_{key.upper()}"
        value = os.getenv(env_name)
        if not value:
            continue
        existing = result.get(key) or BumperPin(key=key, physical_pin=0, label=key)
        result[key] = _apply_one_override(existing, value)
    return result


def _apply_one_override(pin: BumperPin, value: str) -> BumperPin:
    raw = value.strip()
    if ":" in raw:
        chip, line = raw.rsplit(":", 1)
        return replace(pin, chip=chip.strip(), line=int(line.strip()), label=raw)
    return replace(pin, bcm=int(raw), label=f"GPIO{int(raw)}")


def _optional_int(value: Any) -> int | None:
    if value is None or value == "":
        return None
    return int(value)


def _optional_str(value: Any) -> str | None:
    if value is None:
        return None
    text = str(value).strip()
    return text or None


def _read_text(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8", errors="ignore")
    except OSError:
        return ""
