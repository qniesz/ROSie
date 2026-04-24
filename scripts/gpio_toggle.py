"""
Simple GPIO toggle test — no serial needed.
Toggles GPIO17 (physical pin 11) between HIGH and INPUT every 5 seconds.
Measure the Hall sensor signal wire with a multimeter to see the voltage change.

Press Ctrl+C to stop.
"""
import time
import sys

try:
    import RPi.GPIO as GPIO
except ImportError:
    print("RPi.GPIO not found. Install with: pip install RPi.GPIO")
    sys.exit(1)

GPIO_PIN = 17  # BCM pin number (physical pin 11)
TOGGLE_SECS = 5

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(GPIO_PIN, GPIO.IN)

print(f"Toggling GPIO{GPIO_PIN} (physical pin 11) every {TOGGLE_SECS}s")
print("OFF = high-impedance (INPUT), ON = OUTPUT HIGH (3.3V)")
print("Ctrl+C to stop\n")

gpio_on = False
try:
    while True:
        gpio_on = not gpio_on
        if gpio_on:
            GPIO.setup(GPIO_PIN, GPIO.OUT)
            GPIO.output(GPIO_PIN, GPIO.HIGH)
            print(f"[{time.strftime('%H:%M:%S')}] GPIO ON  — 3.3V through resistor", flush=True)
        else:
            GPIO.setup(GPIO_PIN, GPIO.IN)
            print(f"[{time.strftime('%H:%M:%S')}] GPIO OFF — high impedance", flush=True)
        time.sleep(TOGGLE_SECS)
except KeyboardInterrupt:
    print("\nStopping...")
finally:
    GPIO.setup(GPIO_PIN, GPIO.IN)
    GPIO.cleanup()
    print("GPIO cleaned up.")
