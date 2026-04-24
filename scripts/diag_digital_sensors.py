#!/usr/bin/env python3
"""Diagnostic: print raw GetDigitalSensors and GetAnalogSensors output."""
import serial, time

ser = serial.Serial('/dev/neato', 115200, timeout=2)
time.sleep(0.5)
ser.reset_input_buffer()

def send(cmd, wait=1.5):
    ser.reset_input_buffer()
    ser.write((cmd + "\n").encode())
    lines = []
    deadline = time.monotonic() + wait
    while time.monotonic() < deadline:
        line = ser.readline().decode(errors='replace').strip()
        if line:
            lines.append(line)
            if '\x1a' in line or line.endswith('\x1a'):
                break
        if not line and lines:
            break
    return lines

def show(label, lines):
    print(f"\n=== {label} ===")
    for l in lines:
        print(f"  {repr(l)}")

show("GetDigitalSensors", send("GetDigitalSensors"))
show("GetAnalogSensors", send("GetAnalogSensors"))
ser.close()
