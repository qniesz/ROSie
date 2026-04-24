"""
Start a cleaning cycle and monitor for errors with Hall sensor unplugged.
"""
import serial
import time

PORT = '/dev/ttyACM1'

def send(ser, cmd, wait=0.5):
    ser.reset_input_buffer()
    ser.write((cmd + '\n').encode())
    time.sleep(wait)
    return ser.read(ser.in_waiting).decode(errors='replace')

ser = serial.Serial(PORT, 115200, timeout=1)
time.sleep(0.5)
ser.reset_input_buffer()

# Check current state
print("=== Current State ===")
print(send(ser, 'GetState'))

print("=== Current Errors ===")
print(send(ser, 'GetErr'))

print("=== Mag Sensors ===")
print(send(ser, 'GetAnalogSensors'))

# Start cleaning
print("=== Starting House Cleaning ===")
# Need to get serial number for SKey
ver = send(ser, 'GetVersion')
print(ver)

# Try direct SetEvent without SKey first
print("=== Attempting Clean Start ===")
resp = send(ser, 'SetEvent UIMGR_EVENT_SMARTAPP_START_HOUSE_CLEANING', 2)
print(resp)

# Check state after
time.sleep(3)
print("=== State After Start ===")
print(send(ser, 'GetState'))

print("=== Errors After Start ===")
print(send(ser, 'GetErr'))

print("=== Mag Sensors After Start ===")
out = send(ser, 'GetAnalogSensors')
for line in out.split('\n'):
    if 'Mag' in line or 'Error' in line:
        print(line)

# Monitor for 30 seconds
print("\n=== Monitoring for 30 seconds ===")
for i in range(6):
    time.sleep(5)
    state = send(ser, 'GetState')
    err = send(ser, 'GetErr')
    for line in state.split('\n'):
        if 'State' in line or 'Error' in line or 'Alert' in line:
            print(f"[{(i+1)*5}s] {line.strip()}")
    for line in err.split('\n'):
        if line.strip() and not line.startswith('GetErr'):
            print(f"[{(i+1)*5}s] ERR: {line.strip()}")

ser.close()
print("\nDone.")
