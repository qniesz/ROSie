#!/usr/bin/env python3
"""Quick diagnostic: read settings, toggle eco, check nav mode."""
import serial, time

ser = serial.Serial('/dev/ttyACM0', 115200, timeout=2)
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
        # If no data and we already have some lines, we're probably done
        if not line and lines:
            break
    return lines

def show(label, lines):
    print(f"\n=== {label} ===")
    for l in lines:
        print(f"  {l}")

show("Current Settings", send("GetUserSettings"))
show("SetUserSettings EcoMode OFF", send("SetUserSettings EcoMode OFF"))
show("Re-read Settings", send("GetUserSettings"))

time.sleep(5)
show("After 5s delay", send("GetUserSettings"))

show("Help SetNavigationMode", send("Help SetNavigationMode"))
show("GetNavigationMode", send("GetNavigationMode"))

# Try setting Turbo nav mode
show("SetNavigationMode Turbo", send("SetNavigationMode Turbo"))
show("SetNavigationMode Deep", send("SetNavigationMode Deep"))
show("GetNavigationMode after Deep", send("GetNavigationMode"))

# IntenseClean
show("SetUserSettings IntenseClean ON", send("SetUserSettings IntenseClean ON"))
show("Re-read Settings (IntenseClean)", send("GetUserSettings"))

ser.close()
print("\nDone.")
