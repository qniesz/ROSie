#!/usr/bin/env python3
"""
Test script: spin up vacuum motor and monitor RPM while toggling Eco mode.

Must be run on the Pi with the driver stopped:
    sudo systemctl stop rosie-driver
    python3 test_vacuum_eco.py
    sudo systemctl start rosie-driver
"""

import serial
import time
import sys

PORT = "/dev/ttyACM0"
BAUD = 115200


def send(ser, cmd, wait=1.0):
    """Send a command, collect all response lines, return them."""
    ser.reset_input_buffer()
    ser.write((cmd + "\n").encode())
    time.sleep(wait)
    lines = []
    while ser.in_waiting:
        line = ser.readline().decode(errors="replace").strip()
        if line:
            lines.append(line)
    return lines


def get_vacuum_rpm(ser):
    """Read Vacuum_RPM from GetMotors."""
    lines = send(ser, "GetMotors", wait=0.5)
    for line in lines:
        if "Vacuum_RPM" in line:
            parts = line.split(",")
            if len(parts) >= 2:
                return parts[1].strip()
    return "?"


def get_eco_setting(ser):
    """Read EcoMode from GetUserSettings."""
    lines = send(ser, "GetUserSettings", wait=0.5)
    for line in lines:
        if "Eco Mode" in line or "EcoMode" in line:
            parts = line.split(",")
            if len(parts) >= 2:
                return parts[1].strip()
    return "?"


def main():
    print(f"Opening {PORT}...")
    ser = serial.Serial(PORT, BAUD, timeout=1)
    time.sleep(0.5)

    # Drain any startup junk
    ser.reset_input_buffer()

    print("\n=== Initial state ===")
    eco = get_eco_setting(ser)
    print(f"  Eco Mode: {eco}")

    print("\n=== Entering TestMode ===")
    resp = send(ser, "TestMode On")
    for l in resp:
        print(f"  {l}")

    print("\n=== Starting vacuum (SetMotor VacuumOn) ===")
    resp = send(ser, "SetMotor VacuumOn", wait=2.0)
    for l in resp:
        print(f"  {l}")

    # Wait for it to spin up
    print("  Waiting 5s for spin-up...")
    time.sleep(5)

    print("\n=== Reading RPM with current Eco setting ===")
    rpm = get_vacuum_rpm(ser)
    eco = get_eco_setting(ser)
    print(f"  Eco Mode: {eco}  |  Vacuum RPM: {rpm}")

    # Toggle eco OFF
    print("\n=== Setting EcoMode OFF ===")
    resp = send(ser, "SetUserSettings EcoMode OFF")
    for l in resp:
        print(f"  {l}")
    time.sleep(3)

    rpm = get_vacuum_rpm(ser)
    eco = get_eco_setting(ser)
    print(f"  Eco Mode: {eco}  |  Vacuum RPM: {rpm}")

    # Toggle eco ON
    print("\n=== Setting EcoMode ON ===")
    resp = send(ser, "SetUserSettings EcoMode ON")
    for l in resp:
        print(f"  {l}")
    time.sleep(3)

    rpm = get_vacuum_rpm(ser)
    eco = get_eco_setting(ser)
    print(f"  Eco Mode: {eco}  |  Vacuum RPM: {rpm}")

    # Toggle eco OFF again
    print("\n=== Setting EcoMode OFF again ===")
    resp = send(ser, "SetUserSettings EcoMode OFF")
    for l in resp:
        print(f"  {l}")
    time.sleep(3)

    rpm = get_vacuum_rpm(ser)
    eco = get_eco_setting(ser)
    print(f"  Eco Mode: {eco}  |  Vacuum RPM: {rpm}")

    # Continuous monitoring for 15s
    print("\n=== Monitoring RPM for 15s (toggle eco halfway) ===")
    start = time.monotonic()
    toggled = False
    while time.monotonic() - start < 15:
        rpm = get_vacuum_rpm(ser)
        elapsed = time.monotonic() - start
        eco = "?"
        if not toggled and elapsed > 7:
            print(f"  [{elapsed:.1f}s] Toggling EcoMode ON...")
            send(ser, "SetUserSettings EcoMode ON")
            toggled = True
            time.sleep(1)
        print(f"  [{elapsed:.1f}s] Vacuum RPM: {rpm}")
        time.sleep(1)

    # Stop motors and exit TestMode
    print("\n=== Stopping motors ===")
    resp = send(ser, "SetMotor VacuumOff", wait=1.0)
    for l in resp:
        print(f"  {l}")

    print("\n=== Exiting TestMode ===")
    resp = send(ser, "TestMode Off")
    for l in resp:
        print(f"  {l}")

    ser.close()
    print("\nDone. Restart the driver with: sudo systemctl start rosie-driver")


if __name__ == "__main__":
    main()
