#!/usr/bin/env python3
"""Diagnostic: sample MagSensor raw millivolt values with stats."""
import sys
sys.path.insert(0, '/home/rosie')
from rosie_driver.serial_handler import NeatoSerial
import time

s = NeatoSerial(port='/dev/neato')
s.connect()
time.sleep(0.5)

print("=== GetAnalogSensors raw (millivolts) ===")
lines = s.send_and_collect("GetAnalogSensors raw", "SensorName", timeout=2.0)
for l in lines:
    if "Mag" in l or "Wall" in l or "Drop" in l:
        print(f"  {l}")

print("\n=== GetAnalogSensors stats (50-sample avg/max/min) ===")
lines2 = s.send_and_collect("GetAnalogSensors stats", "SensorName", timeout=5.0)
for l in lines2:
    if "Mag" in l or "Wall" in l or "Drop" in l:
        print(f"  {l}")

print("\n=== Native unit values ===")
lines3 = s.send_and_collect("GetAnalogSensors", "SensorName", timeout=2.0)
for l in lines3:
    if "Mag" in l or "Wall" in l or "Drop" in l:
        print(f"  {l}")

s.disconnect()
