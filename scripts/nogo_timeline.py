"""nogo_timeline.py - Pull a clean timeline of no-go events from the Pi.

Usage:
    python scripts/nogo_timeline.py              # last 5 minutes
    python scripts/nogo_timeline.py --tail 2000  # last 2000 lines from container
    python scripts/nogo_timeline.py --follow     # live stream

Categorises log lines into:
    TOUCH       — no_go_guard contact start/end
    PULSE       — pulse sequencer fires (side or front)
    PIN         — GPIO pin LOW/INPUT transitions (hardware-level pulse)
    BUMPER      — Neato firmware reports bumper state via serial
    POS         — periodic robot position log
    CMD         — commands sent to robot (mag_strip, pause_cleaning, etc.)

Each line is shown with a relative timestamp from the first event and a
short coloured tag so the rapid sequence is easy to read.
"""
from __future__ import annotations

import argparse
import os
import re
import subprocess
import sys
from datetime import datetime, timedelta

# Set ROSIE_PI_HOST=user@host in your environment, or override with --pi-host
PI_HOST = os.environ.get("ROSIE_PI_HOST", "rosie@rosie.local")

# Match any of:  "2026-04-18 14:08:50,249 [...] msg"   or   "[INFO] [stamp] [node]: msg"
_TS_RE = re.compile(r"^(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2},\d{3})\s+\[[^\]]+\]\s+\w+:\s*(.*)$")
_RCL_RE = re.compile(r"^\[INFO\]\s+\[(\d+\.\d+)\]\s+\[[^\]]+\]:\s*(.*)$")


def _classify(msg: str) -> tuple[str, str] | None:
    m = msg.strip()
    if not m:
        return None
    low = m.lower()
    if "no-go touch start" in low or "no-go touch release" in low:
        return ("TOUCH ", m)
    if "no_go_guard] touch" in low:
        return ("TOUCH ", m)
    if "nogo pulse #" in low or "no-go pulse phase" in low:
        return ("PULSE ", m)
    if "no-go pulse reset" in low:
        return ("RESET ", m)
    if "[bumper_sensors] pin " in low and ("-> low" in low or "-> input" in low):
        return ("PIN   ", m)
    if "serial bumpers:" in low:
        return ("BUMPER", m)
    if "[no_go_guard] pos=" in low:
        return ("POS   ", m)
    if "executing command:" in low and any(
        c in low for c in ("mag_strip", "pause_cleaning", "house_clean", "send_to_base", "stop_cleaning", "resume_cleaning")
    ):
        return ("CMD   ", m)
    return None


def _parse_ts(line: str) -> tuple[datetime | None, str]:
    m = _TS_RE.match(line)
    if m:
        try:
            ts = datetime.strptime(m.group(1), "%Y-%m-%d %H:%M:%S,%f")
        except ValueError:
            ts = None
        return ts, m.group(2)
    m = _RCL_RE.match(line)
    if m:
        try:
            unix_s = float(m.group(1))
            ts = datetime.fromtimestamp(unix_s)
        except ValueError:
            ts = None
        return ts, m.group(2)
    return None, line.rstrip()


def _format(ts: datetime | None, t0: datetime | None, tag: str, msg: str) -> str:
    if ts is None or t0 is None:
        rel = "      "
    else:
        rel_ms = int((ts - t0).total_seconds() * 1000)
        rel = f"{rel_ms:+6d}ms"
    return f"{rel}  {tag}  {msg}"


def _stream_lines(args: argparse.Namespace):
    cmd = ["ssh", PI_HOST]
    if args.follow:
        cmd.append(f"docker logs -f --tail {args.tail} rosie-driver 2>&1")
    else:
        cmd.append(f"docker logs --tail {args.tail} rosie-driver 2>&1")
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, text=True, bufsize=1, encoding="utf-8", errors="replace")
    assert proc.stdout is not None
    try:
        for line in proc.stdout:
            yield line.rstrip("\n")
    finally:
        proc.terminate()


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--tail", type=int, default=2000, help="lines to fetch from container (default 2000)")
    ap.add_argument("--follow", action="store_true", help="live stream new events")
    ap.add_argument("--minutes", type=float, default=0.0, help="only show events from the last N minutes")
    args = ap.parse_args()

    cutoff = datetime.now() - timedelta(minutes=args.minutes) if args.minutes > 0 else None
    t0: datetime | None = None
    last_ts: datetime | None = None

    for raw in _stream_lines(args):
        ts, msg = _parse_ts(raw)
        cls = _classify(msg)
        if cls is None:
            continue
        tag, text = cls
        if cutoff is not None and ts is not None and ts < cutoff:
            continue
        if t0 is None and ts is not None:
            t0 = ts
        # gap marker if more than 2s passed since last event
        if last_ts is not None and ts is not None and (ts - last_ts).total_seconds() > 2.0:
            print(f"        ----- {(ts-last_ts).total_seconds():.1f}s gap -----")
        if ts is not None:
            last_ts = ts
        print(_format(ts, t0, tag, text))

    return 0


if __name__ == "__main__":
    sys.exit(main())
