"""
Just poll all sensors while robot sits on dock. No cleaning started.
Place/remove magnet near sensors and watch values.
Runs for 60 seconds.
"""
import serial, time

def send(ser, cmd, wait=0.5):
    ser.reset_input_buffer()
    ser.write((cmd + '\n').encode())
    time.sleep(wait)
    return ser.read(ser.in_waiting).decode(errors='replace')

s = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
time.sleep(0.5)
s.reset_input_buffer()

print('Polling ALL analog sensors for 60s. Place/remove magnet to see changes.', flush=True)
print(f'{"Time":>6}  {"MagL":>5}  {"MagR":>5}  {"Wall":>5}  {"DropL":>5}  {"DropR":>5}  {"AccX":>5}  {"AccY":>5}  {"AccZ":>5}', flush=True)
print('-' * 75, flush=True)

t0 = time.time()
for i in range(120):
    out = send(s, 'GetAnalogSensors', 0.4)
    d = {}
    for line in out.split('\n'):
        parts = line.split(',')
        if len(parts) >= 3:
            d[parts[0].strip()] = parts[2].strip()

    elapsed = time.time() - t0
    print(f'{elapsed:5.1f}s  '
          f'{d.get("MagSensorLeft","?"):>5}  '
          f'{d.get("MagSensorRight","?"):>5}  '
          f'{d.get("WallSensor","?"):>5}  '
          f'{d.get("DropSensorLeft","?"):>5}  '
          f'{d.get("DropSensorRight","?"):>5}  '
          f'{d.get("AccelerometerX","?"):>5}  '
          f'{d.get("AccelerometerY","?"):>5}  '
          f'{d.get("AccelerometerZ","?"):>5}',
          flush=True)
    time.sleep(0.1)

s.close()
print('Done.', flush=True)
