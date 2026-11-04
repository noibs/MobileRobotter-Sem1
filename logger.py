import csv
import time

import serial

ser = serial.Serial('/dev/tty.usbmodem1101', 115200, timeout=1)

# Stop whatever is running and soft-reboot, which re-runs main.py
ser.write(b'\x03\x03')   # Ctrl-C twice: interrupt
time.sleep(0.2)
ser.write(b'\x04')       # Ctrl-D: soft reboot
time.sleep(1.5)
ser.reset_input_buffer()

ser.write(b"start\n")

with open('data_log.csv', 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(['t_ms', 'angle_deg', 'adc'])
    while True:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if not line:
            continue
        if line == 'done':
            break
        print(line)
        writer.writerow(line.split(','))
        f.flush()
