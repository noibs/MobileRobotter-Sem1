import csv
import time

import serial

# Open the serial port
# Pathen er anderledes på windows
ser = serial.Serial( '/dev/tty.usbmodem101' , 9600)
# Open a file to write data
with open('data_log.csv' , 'w') as f:
    t0 = time.time()
    writer = csv.writer(f)
    writer.writerow(['time_s', 'adc'])
    while True:
        # Read a line from the serial port
        line = ser.readline().decode( 'utf-8' ).strip()
        print(line) # Print to the terminal
        # Append the data to the file
        writer.writerow([round(time.time() - t0, 3), int(line)])
        f.flush() # Ensure data is written to the file
        time.sleep(1)
