import json
from time import sleep_us

from machine import ADC, Pin


class LineSensor:
    def __init__(self, select_pins=(8, 9, 10), com_pin=26, channels=8, settle_us=20, positions=(-7, -5, -3, -1, 1, 3, 5, 7)):
        if len(select_pins) != 3:
            raise ValueError("CD74HC4051 needs exactly 3 select pins (S0, S1, S2).")
        if not 1 <= channels <= 8:
            raise ValueError("channels must be between 1 and 8.")
        if len(positions) != channels:
            raise ValueError("positions must have one value per channel.")
        # Where each mux channel sits under the robot: negative = left, positive = right, 0 = the middle.
        # Why in robot_config.py: it is a fact about how this robot is wired, not about line following
        self.positions = positions
        self._outermost = max(positions)
        self.channels = channels
        self.settle_us = settle_us  # wait after switching channel before reading the ADC can be turned down we need to calibrate and set as low as possible without having channel values overlapping.
        self._adc = ADC(Pin(com_pin))
        self._select = [Pin(pin, Pin.OUT).value for pin in select_pins]
        self.values = [0] * channels
        self.line_count = 0  # sensors that saw the line in the last read_position()
        # None if calibrate() has never been run
        self._set_calibration(*self._load_calibration())

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

    def read_position(self, noise_floor=100):
        # Where the line is: -1.0 = far left, 0 = middle, 1.0 = far right, None if no sensor sees it.
        # Also sets self.line_count = how many sensors see the line. Why: many at once = a crossing or T.
        # Why not just a threshold (line / no line): the in-between values tell WHERE between two sensors
        # the line is, so the position moves smoothly instead of jumping from sensor to sensor
        values = self.read_raw()
        low = self.min_value
        span = self._span
        positions = self.positions
        weighted = 0
        total = 0
        count = 0
        for i in range(self.channels):
            # Scale to 0 = floor, 1000 = line. Why per sensor: each sensor reads floor and line differently,
            # so raw values from two sensors can't be compared. Why ints: faster than floats in MicroPython
            value = (values[i] - low[i]) * 1000 // span[i]
            if value < noise_floor:
                continue  # why: just above the floor is noise, not the line
            if value > 1000:
                value = 1000
            # Weighted average of where the line is. Why the positions are 0 in the middle: the error
            # is then 0 when the robot is centred on the line
            weighted += value * positions[i]
            total += value
            count += 1
        self.line_count = count
        if total == 0:
            return None
        # We divide by the outermost position to normalise the position between -1 and 1 no matter the amount of sensors
        return weighted / (total * self._outermost)

    def calibrate(self, robot, distance_cm=15, speed_cm_s=6):
        # Drives distance_cm forward and back over the line and keeps each sensor's lowest (floor) and
        # highest (line) value. Why: every sensor reads differently, and read_position() needs each
        # sensor's own floor and line value to scale them all to the same 0-1000
        self._cal_min = [65535] * self.channels
        self._cal_max = [0] * self.channels
        self._cal_direction = 1.0
        robot.move(self._read_and_drive_straight, speed_cm_s, distance_cm)
        self._cal_direction = -1.0  # why back again: so the robot ends where it started
        robot.move(self._read_and_drive_straight, speed_cm_s, distance_cm)
        min_values, max_values = self._cal_min, self._cal_max

        # A sensor that never saw both line and floor would divide by ~0 later
        for i in range(self.channels):
            if max_values[i] - min_values[i] < 2000:
                raise ValueError("Sensor %d did not see both line and floor: %d-%d" % (i, min_values[i], max_values[i]))

        with open("calibration.json", "w") as f:
            json.dump({"min": min_values, "max": max_values}, f)
        self._set_calibration(min_values, max_values)

    def _read_and_drive_straight(self):
        # robot.move() calls this on core 1 about every 1 ms while calibrating.
        # Why read here: we get values from every point the robot passes, not just where it stops
        values = self.read_raw()
        for i in range(self.channels):
            self._cal_min[i] = min(self._cal_min[i], values[i])
            self._cal_max[i] = max(self._cal_max[i], values[i])
        return self._cal_direction, self._cal_direction

    def _set_calibration(self, min_values, max_values):
        self.min_value, self.max_value = min_values, max_values
        # max - min per sensor, calculated once so read_position() doesn't do it every read
        if min_values is None:
            self._span = None
        else:
            self._span = [high - low for low, high in zip(min_values, max_values)]

    def _load_calibration(self):
        try:
            with open("calibration.json") as f:
                data = json.load(f)
            return data["min"], data["max"]
        except (OSError, ValueError, KeyError):
            # OSError: no file yet. ValueError/KeyError: the file is broken
            print("Line sensor needs to be calibrated first, run calibrate.py.")
            return None, None
