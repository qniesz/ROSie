"""
GPIO Mag Sensor Injection Test
==============================
Toggles a Pi GPIO pin HIGH/LOW while monitoring MagSensor raw mV values.
Connect: GPIO pin --> resistor --> Hall sensor signal wire

Usage:
  1. Wire GPIO pin (default: GPIO17, physical pin 11) through a resistor
     to the Hall sensor signal wire
  2. Run this script on the Pi
  3. Watch mV values — GPIO ON should push voltage from ~1000mV toward ~1300mV+
  4. If scaled value hits 2, that resistor works. If not, try smaller resistor.

Press Ctrl+C to stop.
"""
import serial
import time
import sys

try:
    import RPi.GPIO as GPIO
except ImportError:
    print("RPi.GPIO not found. Install with: pip install RPi.GPIO")
    sys.exit(1)

# --- Config ---
GPIO_PIN = 17          # BCM pin number (physical pin 11)
SERIAL_PORT = '/dev/ttyACM0'
TOGGLE_SECS = 15       # seconds between on/off toggles
POLL_HZ = 2            # sensor reads per second


def send(ser, cmd, wait=0.4):
    ser.reset_input_buffer()
    ser.write((cmd + '\n').encode())
    time.sleep(wait)
    return ser.read(ser.in_waiting).decode(errors='replace')


def parse_sensors(output):
    vals = {}
    for line in output.split('\n'):
        parts = line.split(',')
        if len(parts) >= 3:
            vals[parts[0].strip()] = parts[2].strip()
    return vals


def main():
    # Setup GPIO
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    # Start as INPUT (high-impedance = no effect on sensor)
    GPIO.setup(GPIO_PIN, GPIO.IN)

    # Open serial
    ser = serial.Serial(SERIAL_PORT, 115200, timeout=1)
    time.sleep(0.5)
    ser.reset_input_buffer()

    gpio_on = False
    last_toggle = time.time()

    print(f'GPIO Mag Sensor Test — pin GPIO{GPIO_PIN} (physical pin 11)')
    print(f'ON = OUTPUT LOW (pull to 0V = detection), OFF = OUTPUT HIGH (pull to 3.3V = no detection)')
    print(f'Toggling every {TOGGLE_SECS}s. Ctrl+C to stop.\n')
    print(f'{"Time":>6}  {"GPIO":>5}  {"MagL_mV":>8}  {"MagR_mV":>8}  {"MagL":>5}  {"MagR":>5}')
    print('-' * 52)

    t0 = time.time()
    try:
        while True:
            now = time.time()

            # Toggle GPIO every TOGGLE_SECS
            if now - last_toggle >= TOGGLE_SECS:
                gpio_on = not gpio_on
                if gpio_on:
                    # Switch to OUTPUT LOW — pulls signal toward 0V through resistor
                    GPIO.setup(GPIO_PIN, GPIO.OUT)
                    GPIO.output(GPIO_PIN, GPIO.LOW)
                else:
                    # Switch to OUTPUT HIGH — pulls signal toward 3.3V (no detection)
                    GPIO.setup(GPIO_PIN, GPIO.OUT)
                    GPIO.output(GPIO_PIN, GPIO.HIGH)
                last_toggle = now

            # Read raw mV
            raw = parse_sensors(send(ser, 'GetAnalogSensors raw', 0.3))
            # Read scaled
            scaled = parse_sensors(send(ser, 'GetAnalogSensors', 0.2))

            elapsed = now - t0
            state = " ON" if gpio_on else "OFF"
            ml_raw = raw.get('MagSensorLeft', '?')
            mr_raw = raw.get('MagSensorRight', '?')
            ml_sc = scaled.get('MagSensorLeft', '?')
            mr_sc = scaled.get('MagSensorRight', '?')

            print(f'{elapsed:5.1f}s  {state:>5}  {ml_raw:>8}  {mr_raw:>8}  {ml_sc:>5}  {mr_sc:>5}',
                  flush=True)

            time.sleep(1.0 / POLL_HZ)

    except KeyboardInterrupt:
        print('\nStopping...')
    finally:
        GPIO.setup(GPIO_PIN, GPIO.OUT)
        GPIO.output(GPIO_PIN, GPIO.HIGH)  # leave HIGH (no detection) on exit
        GPIO.cleanup()
        ser.close()
        print('GPIO cleaned up, serial closed.')


if __name__ == '__main__':
    main()
