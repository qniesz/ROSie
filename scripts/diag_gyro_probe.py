"""Probe the Neato D6 for gyro / accelerometer / IMU capability."""
import serial, time, sys

s = serial.Serial('/dev/ttyACM0', 115200, timeout=1.5)


def send(cmd, wait=0.4):
    s.reset_input_buffer()
    s.write((cmd + '\n').encode())
    time.sleep(wait)
    return s.read_all().decode(errors='ignore')


for cmd in [
    'Help',
    'GetAccel',
    'Help GetAccel',
    'GetGyro',
    'Help GetGyro',
    'GetIMU',
    'GetMotors',
]:
    print(f'=== {cmd} ===')
    print(send(cmd)[:1500])
    print()
