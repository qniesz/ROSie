#!/usr/bin/env python3
"""Check if we can raise vacuum power above 65%."""
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
            if '\x1a' in line:
                break
        if not line and lines:
            break
    return lines

def show(label, lines):
    print(f"\n=== {label} ===")
    for l in lines:
        print(f"  {l}")

# Check Help SetUsage
show("Help SetUsage", send("Help SetUsage"))

# Enter TestMode to try SetMotor with speed
show("TestMode On", send("TestMode On"))

# Check Help SetMotor
show("Help SetMotor", send("Help SetMotor", wait=3))

# Try vacuum at different speeds
show("SetMotor VacuumSpeed 80 VacuumOn", send("SetMotor VacuumSpeed 80 VacuumOn", wait=2))
time.sleep(3)

# Read RPM at 80%
show("GetMotors (80%)", send("GetMotors"))

# Try 100%
show("SetMotor VacuumSpeed 100 VacuumOn", send("SetMotor VacuumSpeed 100 VacuumOn", wait=2))
time.sleep(3)
show("GetMotors (100%)", send("GetMotors"))

# Stop
show("SetMotor VacuumOff", send("SetMotor VacuumOff"))

# Try SetUsage VacuumPwr
show("SetUsage VacuumPwr 80", send("SetUsage VacuumPwr 80"))
show("SetUsage VacuumPwr 100", send("SetUsage VacuumPwr 100"))

show("TestMode Off", send("TestMode Off"))
ser.close()
print("\nDone.")
