#!/usr/bin/env python3
"""Diagnostic: print ALL raw GetDigitalSensors and GetAnalogSensors field names."""
import sys
sys.path.insert(0, '/home/rosie')
from rosie_driver.serial_handler import NeatoSerial
import time

s = NeatoSerial(port='/dev/neato')
s.connect()
time.sleep(0.5)

print("=== GetDigitalSensors ===")
lines = s.send_and_collect("GetDigitalSensors", "Digital Sensor Name", timeout=2.0)
for l in lines:
    print(f"  {repr(l)}")

print("\n=== GetAnalogSensors ===")
lines2 = s.send_and_collect("GetAnalogSensors", "SensorName", timeout=2.0)
for l in lines2:
    print(f"  {repr(l)}")

s.disconnect()
