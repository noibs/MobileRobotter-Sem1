from time import sleep_ms, ticks_diff, ticks_us

from line_sensor import LineSensor

sensor = LineSensor()

# Prints how long one read of all sensors takes, and the raw values
while True:
    start = ticks_us()
    values = sensor.read_raw()
    elapsed = ticks_diff(ticks_us(), start)
    print(elapsed, "us", values)
    sleep_ms(200)
