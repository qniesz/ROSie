"""
scan_recorder.py — record raw LidarScan + OdomState to JSONL for offline replay.

Used to capture real cleaning-cycle data so we can compare SLAM engines
(BreezySLAM vs karto_sdk) on identical input without re-running on the robot.

File format: one JSON object per line.

    {"t":<scan timestamp>,
     "scan":{"angle_min":..,"angle_inc":..,"ranges":[..],"intensities":[..]},
     "odom":{"x":..,"y":..,"theta":..,"lin":..,"ang":..,"t":..}}

Output dir: ~/scan_logs/  (created on first start)
File name:  scan_<YYYYmmdd-HHMMSS>.jsonl
Size cap:   250 MB → auto-stop to protect SD card

Thread-safety: a single Lock guards _file.  start/stop are idempotent.
"""

from __future__ import annotations

import json
import logging
import math
import os
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Optional

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Tuning
# ---------------------------------------------------------------------------
DEFAULT_DIR = Path(os.environ.get(
    "ROSIE_SCAN_LOG_DIR", str(Path.home() / "scan_logs")
))
MAX_BYTES = int(os.environ.get("ROSIE_SCAN_LOG_MAX_BYTES", 250 * 1024 * 1024))


# ---------------------------------------------------------------------------
# Module state
# ---------------------------------------------------------------------------
@dataclass
class _State:
    file: Optional[Any] = None        # text file handle
    path: Optional[Path] = None
    started_at: float = 0.0
    scans: int = 0
    bytes_written: int = 0
    auto_stopped: bool = False        # tripped when MAX_BYTES exceeded

_state = _State()
_lock = threading.Lock()


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------
def start(directory: Optional[Path] = None) -> dict:
    """Open a fresh log file.  Returns status dict.  Idempotent: a second
    start() while already recording returns the current status unchanged."""
    with _lock:
        if _state.file is not None:
            logger.info("scan_recorder.start ignored — already recording %s",
                        _state.path)
            return _status_locked()

        d = Path(directory) if directory else DEFAULT_DIR
        d.mkdir(parents=True, exist_ok=True)
        ts = time.strftime("%Y%m%d-%H%M%S", time.localtime())
        path = d / f"scan_{ts}.jsonl"
        try:
            _state.file = open(path, "w", buffering=8192, encoding="utf-8")
        except OSError as exc:
            logger.error("scan_recorder: cannot open %s: %s", path, exc)
            return {"state": "error", "error": str(exc)}

        _state.path = path
        _state.started_at = time.monotonic()
        _state.scans = 0
        _state.bytes_written = 0
        _state.auto_stopped = False
        logger.info("scan_recorder: started → %s", path)
        return _status_locked()


def stop() -> dict:
    """Close the log file.  Returns final status dict (state='idle')."""
    with _lock:
        if _state.file is None:
            return _status_locked()
        try:
            _state.file.flush()
            _state.file.close()
        except Exception:  # noqa: BLE001
            logger.debug("scan_recorder: close error", exc_info=True)
        logger.info(
            "scan_recorder: stopped — %d scans, %d bytes → %s",
            _state.scans, _state.bytes_written, _state.path,
        )
        _state.file = None
        return _status_locked()


def record(scan, odom) -> None:
    """Append one (scan, odom) pair.  Cheap no-op when not recording.

    Called from the main serial loop at ~5 Hz, so this stays small:
    one JSON serialise, one write.  Buffered I/O flushes lazily.
    """
    if _state.file is None:
        return

    # Build the row before taking the lock — JSON encode dominates cost
    # and is pure-Python; no need to serialise other recorders.
    try:
        row = {
            "t": float(getattr(scan, "timestamp", 0.0) or 0.0),
            "scan": {
                "angle_min": float(getattr(scan, "angle_min", 0.0)),
                "angle_inc": float(getattr(
                    scan, "angle_increment", math.pi / 180.0,
                )),
                # Inf → null so JSON stays valid; replay treats null as miss.
                "ranges": [
                    (None if not math.isfinite(r) else float(r))
                    for r in scan.ranges
                ],
                "intensities": [
                    float(v) for v in getattr(scan, "intensities", [])
                ],
            },
            "odom": None if odom is None else {
                "x":     float(odom.x),
                "y":     float(odom.y),
                "theta": float(odom.theta),
                "lin":   float(getattr(odom, "linear_vel",  0.0)),
                "ang":   float(getattr(odom, "angular_vel", 0.0)),
                "t":     float(odom.timestamp),
            },
        }
        line = json.dumps(row, separators=(",", ":")) + "\n"
    except Exception:  # noqa: BLE001
        logger.debug("scan_recorder: serialise failed", exc_info=True)
        return

    with _lock:
        f = _state.file
        if f is None:
            return
        try:
            f.write(line)
        except Exception:  # noqa: BLE001
            logger.error("scan_recorder: write failed; auto-stopping",
                         exc_info=True)
            try:
                f.close()
            except Exception:
                pass
            _state.file = None
            return

        _state.scans += 1
        _state.bytes_written += len(line)

        if _state.bytes_written >= MAX_BYTES and not _state.auto_stopped:
            logger.warning(
                "scan_recorder: %d bytes ≥ cap %d — auto-stopping",
                _state.bytes_written, MAX_BYTES,
            )
            _state.auto_stopped = True
            try:
                f.flush()
                f.close()
            except Exception:
                pass
            _state.file = None


def is_recording() -> bool:
    return _state.file is not None


def get_status() -> dict:
    with _lock:
        return _status_locked()


# ---------------------------------------------------------------------------
# Internal
# ---------------------------------------------------------------------------
def _status_locked() -> dict:
    """Caller must hold _lock."""
    if _state.file is not None:
        duration = time.monotonic() - _state.started_at
        return {
            "state":      "recording",
            "path":       str(_state.path) if _state.path else "",
            "scans":      _state.scans,
            "bytes":      _state.bytes_written,
            "duration_s": round(duration, 1),
        }
    return {
        "state":        "idle",
        "path":         str(_state.path) if _state.path else "",
        "scans":        _state.scans,
        "bytes":        _state.bytes_written,
        "duration_s":   0.0,
        "auto_stopped": _state.auto_stopped,
    }
