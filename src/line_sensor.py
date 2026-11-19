import json
from time import sleep_us

from machine import ADC, Pin


class LineSensor:
    def __init__(self, select_pins=(8, 9, 10), com_pin=26, channels=8, settle_us=20):
        if len(select_pins) != 3:
            raise ValueError("CD74HC4051 needs exactly 3 select pins (S0, S1, S2).")
        if not 1 <= channels <= 8:
            raise ValueError("channels must be between 1 and 8.")
        self.channels = channels
        self.settle_us = settle_us  # wait after switching channel before reading the ADC can be turned down we need to calibrate and set as low as possible without having channel values overlapping.
        self._adc = ADC(Pin(com_pin))
        self._select = [Pin(pin, Pin.OUT).value for pin in select_pins]
        self.values = [0] * channels
        # None if calibrate() has never been run
        self.min_value, self.max_value = self._load_calibration()

    def read_raw(self):
        s0, s1, s2 = self._select
        read = self._adc.read_u16
        values = self.values
        for channel in range(self.channels):
            # The 3 bits of the channel number pick which input the mux connects to COM
            s0(channel & 1)
            s1((channel >> 1) & 1)
            s2((channel >> 2) & 1)
            sleep_us(self.settle_us)
            values[channel] = read()
        return values

    def calibrate(self, robot, distance_cm=15, speed_cm_s=2):
        min_values = [65535] * self.channels
        max_values = [0] * self.channels
        steps = round(distance_cm / robot.distance_per_step)
        count = 0

        def read_while_driving():
            # drive_while calls this before every step and stops when it returns False
            nonlocal count
            values = self.read_raw()
            for i in range(self.channels):
                min_values[i] = min(min_values[i], values[i])
                max_values[i] = max(max_values[i], values[i])
            count += 1
            return count <= steps

        # start speed = speed means no ramp, so it stops exactly when the distance is driven
        robot.drive_while(read_while_driving, "forward", speed_cm_s=speed_cm_s, start_speed_cm_s=speed_cm_s, check_every=1)
        count = 0
        robot.drive_while(read_while_driving, "backward", speed_cm_s=speed_cm_s, start_speed_cm_s=speed_cm_s, check_every=1)

        # A sensor that never saw both line and floor would divide by ~0 later
        for i in range(self.channels):
            if max_values[i] - min_values[i] < 2000:
                raise ValueError("Sensor %d did not see both line and floor: %d-%d" % (i, min_values[i], max_values[i]))

        with open("calibration.json", "w") as f:
            json.dump({"min": min_values, "max": max_values}, f)
        self.min_value, self.max_value = min_values, max_values

    def _load_calibration(self):
        try:
            with open("calibration.json") as f:
                data = json.load(f)
            return data["min"], data["max"]
        except (OSError, ValueError, KeyError):
            # OSError: no file yet. ValueError/KeyError: the file is broken
            print("Line sensor needs to be calibrated first, run calibrate.py.")
            return None, None
