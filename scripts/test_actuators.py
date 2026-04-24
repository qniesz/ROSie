import serial, time

s = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
s.flushInput()
s.write(b'\n\n\n')
time.sleep(0.2)
s.flushInput()

s.write(b'TestMode On\n')
time.sleep(0.5)
s.read(4096)

# === TEST 1: Main Brush ===
print('=== MAIN BRUSH (3 seconds) ===')
s.write(b'SetMotor Brush RPM 1200\n')
time.sleep(3)
s.read(4096)
s.write(b'SetMotor Brush RPM 0\n')
time.sleep(0.3)
s.read(4096)
print('  Brush stopped.')

# === TEST 2: Vacuum Motor ===
print('=== VACUUM MOTOR (3 seconds) ===')
s.write(b'SetMotor VacuumOn\n')
time.sleep(3)
s.read(4096)
s.write(b'SetMotor VacuumOff\n')
time.sleep(0.3)
s.read(4096)
print('  Vacuum stopped.')

# === TEST 3: Side Brush ===
print('=== SIDE BRUSH (3 seconds) ===')
s.write(b'SetMotor SideBrushEnable\n')
time.sleep(0.3)
s.read(4096)
s.write(b'SetMotor SideBrushPower 5000\n')
time.sleep(3)
s.read(4096)
s.write(b'SetMotor SideBrushPower 0\n')
time.sleep(0.3)
s.write(b'SetMotor SideBrushDisable\n')
time.sleep(0.3)
s.read(4096)
print('  Side brush stopped.')

# === TEST 4: Wall/Side Distance Sensor ===
print('=== ANALOG SENSORS ===')
s.write(b'GetAnalogSensors\n')
time.sleep(0.5)
data = s.read(8192)
text = data.decode('ascii', errors='replace')
for line in text.split('\n'):
    line = line.strip()
    if line and ',' in line:
        print('  %s' % line)

# Cleanup
s.write(b'TestMode Off\n')
time.sleep(0.2)
s.close()
print('\nDone.')
