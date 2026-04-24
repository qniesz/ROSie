import serial, time

s = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
time.sleep(0.5)
s.reset_input_buffer()

def send(cmd, wait=0.5):
    s.reset_input_buffer()
    s.write((cmd + '\n').encode())
    time.sleep(wait)
    return s.read(s.in_waiting).decode(errors='replace')

for cmd in ['Help GetSensor', 'Help SetSensor', 'GetSensor', 'GetSensor Wall', 'GetSensor UltraSound', 'GetSensor Mag']:
    print(f'>>> {cmd}')
    print(send(cmd, 0.5))
    print()

s.close()
