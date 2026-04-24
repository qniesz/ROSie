"""
Bump Sensor GPIO Read Test
===========================
Polls all 4 physical bump switch GPIOs and prints a live status table.
Run standalone on the Pi (driver does NOT need to be running).

Pin mapping (matches bumper_sensors.py):
  GPIO5  (physical pin 29) — Front Left
  GPIO6  (physical pin 31) — Front Right
  GPIO13 (physical pin 33) — Side Left
  GPIO26 (physical pin 37) — Side Right

Pins are configured with NO pull-up/pull-down (floating).
Pressing a button should pull the pin to GND (LOW / ~0V).
With no button pressed the pin floats (reads unpredictably).

Usage:
  python scripts/bump_test.py

Press Ctrl+C to stop.
"""

import sys
import time

try:
    import RPi.GPIO as GPIO
except ImportError:
    print("RPi.GPIO not found. Run this on the Pi.")
    sys.exit(1)

SENSORS = [
    ("Front Left",  5),
    ("Front Right", 6),
    ("Side Left",  13),
    ("Side Right", 26),
]

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
for _, pin in SENSORS:
    GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

print("Bump Sensor GPIO Read Test")
print("PUD_UP — HIGH when idle, LOW when pressed")
print("Press Ctrl+C to stop.\n")

header = "  ".join(f"{name:>12}" for name, _ in SENSORS)
print(header)
print("-" * len(header))

try:
    while True:
        vals = []
        for name, pin in SENSORS:
            level = GPIO.input(pin)
            label = "HIGH" if level == GPIO.HIGH else "LOW"
            vals.append(f"{label:>12}")
        print("  ".join(vals), end="\r", flush=True)
        time.sleep(0.5)
except KeyboardInterrupt:
    print("\n\nStopped.")
finally:
    GPIO.cleanup()
