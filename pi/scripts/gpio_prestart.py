#!/usr/bin/env python3
"""gpio_prestart.py — Hold bumper GPIO lines as floating INPUT during Pi boot.

Runs as rosie-gpio-prestart.service (DefaultDependencies=no) so it starts
as early as possible after the GPIO driver is up.  Keeps the 4 bumper lines
claimed as INPUT/no-pull so they stay HIGH (via the Neato's own pull-ups)
throughout the Linux boot window.

Stopped by rosie.service ExecStartPre before rosie claims the same lines.
"""
import gpiod
import signal
import sys
import logging

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s gpio_prestart %(message)s",
    stream=sys.stdout,
)
log = logging.getLogger()

CHIP = "/dev/gpiochip1"
# PI0/pin29 (front_left), PI12/pin33 (side_left), PH4/pin18 (front_right), PC12/pin36 (side_right)
# front_right+side_right intentionally on different banks (PH vs PC) to prevent simultaneous LOW at boot
LINES = [256, 268, 228, 76]

line_mod = gpiod.line
settings = gpiod.LineSettings(
    direction=line_mod.Direction.INPUT,
    bias=line_mod.Bias.DISABLED,
)
config = {line: settings for line in LINES}

try:
    req = gpiod.request_lines(CHIP, consumer="rosie-prestart", config=config)
    log.info("Lines %s claimed as INPUT/no-pull on %s", LINES, CHIP)
except Exception as e:
    # Don't block boot if this fails — just let rosie.service handle it
    log.error("Failed to claim GPIO lines: %s — will not block boot", e)
    sys.exit(0)


def _release(signum, frame):
    try:
        req.release()
        log.info("Lines released, exiting")
    except Exception:
        pass
    sys.exit(0)


signal.signal(signal.SIGTERM, _release)
signal.signal(signal.SIGINT, _release)
signal.pause()
