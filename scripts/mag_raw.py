"""
Poll MagSensor raw mV values. Hold magnet near sensor to see raw threshold.
"""
import serial, time

def send(ser, cmd, wait=0.4):
    ser.reset_input_buffer()
    ser.write((cmd + '\n').encode())
    time.sleep(wait)
    return ser.read(ser.in_waiting).decode(errors='replace')

s = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
time.sleep(0.5)
s.reset_input_buffer()

print('Polling raw MagSensor mV for 60s. Place/remove magnet to see changes.', flush=True)
print(f'{"Time":>6}  {"MagL_mV":>8}  {"MagR_mV":>8}  {"MagL_scaled":>11}  {"MagR_scaled":>11}', flush=True)
print('-' * 55, flush=True)

t0 = time.time()
for i in range(120):
    # Get raw mV
    out_raw = send(s, 'GetAnalogSensors raw', 0.3)
    raw = {}
    for line in out_raw.split('\n'):
        parts = line.split(',')
        if len(parts) >= 3:
            raw[parts[0].strip()] = parts[2].strip()

    # Get scaled
    out_scaled = send(s, 'GetAnalogSensors', 0.2)
    scaled = {}
    for line in out_scaled.split('\n'):
        parts = line.split(',')
        if len(parts) >= 3:
            scaled[parts[0].strip()] = parts[2].strip()

    elapsed = time.time() - t0
    ml_raw = raw.get('MagSensorLeft', '?')
    mr_raw = raw.get('MagSensorRight', '?')
    ml_sc = scaled.get('MagSensorLeft', '?')
    mr_sc = scaled.get('MagSensorRight', '?')
    
    print(f'{elapsed:5.1f}s  {ml_raw:>8}  {mr_raw:>8}  {ml_sc:>11}  {mr_sc:>11}', flush=True)
    time.sleep(0.1)

s.close()
print('Done.', flush=True)
