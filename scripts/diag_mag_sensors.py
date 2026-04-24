#!/usr/bin/env python3
"""Diagnostic: query Neato help and analog sensor details."""
import sys
sys.path.insert(0, '/home/rosie')
from rosie_driver.serial_handler import NeatoSerial
import time

s = NeatoSerial(port='/dev/neato')
s.connect()
time.sleep(0.5)

# Sample MagSensor values multiple times
print("=== MagSensor readings (10 samples, 0.5s apart) ===")
for i in range(10):
    lines = s.send_and_collect("GetAnalogSensors", "SensorName", timeout=2.0)
    for l in lines:
        if "MagSensor" in l or "WallSensor" in l or "DropSensor" in l:
            print(f"  {l}")
    time.sleep(0.5)

print("\n=== GetDigitalSensors raw ===")
lines = s.send_and_collect("GetDigitalSensors", "Digital Sensor Name", timeout=2.0)
for l in lines:
    print(f"  {repr(l)}")

print("\n=== Help GetAnalogSensors ===")
s.flush()
s.send_command("Help GetAnalogSensors")
import time as _t; _t.sleep(1.5)
lines2 = []
while True:
    l, last = s.get_response(timeout=1.0)
    if not l:
        break
    lines2.append(l)
    if last:
        break
for l in lines2:
    print(f"  {l}")

s.disconnect()
