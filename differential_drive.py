import gc
import math
from time import sleep_us, ticks_add, ticks_diff, ticks_us

from stepper_motor import StepperMotor

# Highest PWM allowed without ramping.
MAX_UNRAMPED_PWM_PCT = 0.2


class DifferentialDrive:
    def __init__(
        self,
        left,
        right,
        wheel_diameter_cm=8.65,
        wheelbase_cm=23.7,
        invert_left=False,
        invert_right=False,
        speed_cm_s=15,
        start_speed_cm_s=4,
        acceleration_cm_s2=10,
        min_pwm_pct=0.1,
        max_pwm_pct=0.2,
    ):
        # Geometry is in cm and calculated once here instead of in every method
        self.left_motor = left
        self.right_motor = right
        self.wheel_diameter_cm = wheel_diameter_cm
        self.wheelbase_cm = wheelbase_cm
        self.wheel_circumference_cm = math.pi * wheel_diameter_cm
        self.steps_per_revolution = 200 * len(left.step_sequence) // 4
        self.distance_per_step = self.wheel_circumference_cm / self.steps_per_revolution
        self.speed_delay = self.speed_to_delay_us(speed_cm_s)
        self.start_speed_delay = self.speed_to_delay_us(start_speed_cm_s)
        self._check_acceleration(acceleration_cm_s2)
        self.acceleration_cm_s2 = acceleration_cm_s2
        self._check_pwm(min_pwm_pct, max_pwm_pct)
        # Stored as pwm_scale values (16384 = 100%), so the drive loop needs no conversion
        self.min_pwm_scale = int(min_pwm_pct * 16384)
        self.max_pwm_scale = int(max_pwm_pct * 16384)
        # -1 flips a motor that is mounted mirrored, so "forward" drives the wheel forward
        self.left_sign = -1 if invert_left else 1
        self.right_sign = -1 if invert_right else 1
        self.stop()

    def speed_to_delay_us(self, speed_cm_s):
        # cm/s -> steps/s -> microseconds between two steps
        if speed_cm_s <= 0:
            raise ValueError("Speed must be greater than 0.")
        steps_per_second = speed_cm_s / self.distance_per_step

        return int(1_000_000 / steps_per_second)

    def _check_acceleration(self, acceleration_cm_s2):
        if acceleration_cm_s2 <= 0:
            raise ValueError("Acceleration must be greater than 0.")

    # We make sure the pwm_pct is between 0 and 1, and that min is lower than max
    def _check_pwm(self, min_pwm_pct, max_pwm_pct):
        if not 0 < min_pwm_pct <= 1 or not 0 < max_pwm_pct <= 1:
            raise ValueError("PWM must be between 0 and 1.")
        if min_pwm_pct > MAX_UNRAMPED_PWM_PCT:
            raise ValueError(
                "min_pwm_pct must not be above %s without ramping." % MAX_UNRAMPED_PWM_PCT
            )
        if min_pwm_pct > max_pwm_pct:
            raise ValueError("min_pwm_pct must not be bigger than max_pwm_pct.")

    def _ramp_steps_needed(self, start_speed_delay, speed_delay, acceleration_cm_s2):
        # Steps needed to reach full speed: s = (v^2 - v0^2) / (2 * a), in steps and steps/s
        start_speed = 1_000_000 / start_speed_delay  # steps per second
        target_speed = 1_000_000 / speed_delay
        acceleration = acceleration_cm_s2 / self.distance_per_step  # steps per second^2
        return int((target_speed**2 - start_speed**2) / (2 * acceleration))

    def _ramp_delays(
        self, start_speed_delay, speed_delay, acceleration_cm_s2, ramp_steps
    ):
        # Calculated once before the move, so no float math happens while stepping
        start_speed = 1_000_000 / start_speed_delay  # steps per second
        target_speed = 1_000_000 / speed_delay
        acceleration = acceleration_cm_s2 / self.distance_per_step  # steps per second^2

        delays, pwms = [], []
        for i in range(ramp_steps):
            # Constant acceleration: v^2 = v0^2 + 2 * a * s
            speed = math.sqrt(start_speed**2 + 2 * acceleration * i)
            delays.append(int(1_000_000 / speed))

            # At low speeds the motor needs less current, so PWM rises with speed.
            fraction = (speed - start_speed) / (target_speed - start_speed)
            pwms.append(
                int(
                    self.min_pwm_scale
                    + (self.max_pwm_scale - self.min_pwm_scale) * fraction
                )
            )
        """
        print("Pwms:", pwms)
        print("---------------------")
        print("Delays:", delays)
        print("---------------------")
        """
        return delays, pwms

    def _run_steps(
        self,
        left_steps,
        right_steps,
        left_direction,
        right_direction,
        speed_cm_s=None,
        start_speed_cm_s=None,
        acceleration_cm_s2=None,
    ):
        if left_steps < 0 or right_steps < 0:
            raise ValueError("Steps must be non-negative.")
        if speed_cm_s is None:
            speed_delay = self.speed_delay
        else:
            speed_delay = self.speed_to_delay_us(speed_cm_s)
        if start_speed_cm_s is None:
            start_speed_delay = self.start_speed_delay
        else:
            start_speed_delay = self.speed_to_delay_us(start_speed_cm_s)
        if acceleration_cm_s2 is None:
            acceleration_cm_s2 = self.acceleration_cm_s2
        else:
            self._check_acceleration(acceleration_cm_s2)

        # Directions are 1 (forward) or -1 (backward), flipped if mounted mirrored
        left_step = left_direction * self.left_sign
        right_step = right_direction * self.right_sign

        speed_delay = max(
            speed_delay, self.left_motor.min_delay_us, self.right_motor.min_delay_us
        )

        start_speed_delay = max(start_speed_delay, speed_delay)

        total_steps = max(left_steps, right_steps)

        # A short move only uses the part of the ramp it has room for (at most half the move)
        ramp_needed = self._ramp_steps_needed(
            start_speed_delay, speed_delay, acceleration_cm_s2
        )
        ramp = min(ramp_needed, total_steps // 2)
        ramp_delays, ramp_pwms = self._ramp_delays(
            start_speed_delay, speed_delay, acceleration_cm_s2, ramp
        )

        # Between the two ramps: full speed, or the top speed a short move could reach.
        # No ramp at all (ramp_needed == 0) goes to else: min PWM, never above the unramped limit
        if ramp_needed > 0 and ramp == ramp_needed:
            middle_delay = speed_delay
            middle_pwm = self.max_pwm_scale
        elif ramp > 0:
            middle_delay = ramp_delays[-1]
            middle_pwm = ramp_pwms[-1]
        else:
            middle_delay = start_speed_delay
            middle_pwm = self.min_pwm_scale
        gc.collect()

        # Saved so the motors get their normal strength back after the move
        left_scale, right_scale = self.left_motor.pwm_scale, self.right_motor.pwm_scale
        try:
            # Runs until the wheel with most steps is done, the other stops when its steps are used
            next_time = ticks_us()
            for index in range(max(left_steps, right_steps)):
                # How many steps to the closest end of the move (start or end)
                nearest = min(index, total_steps - 1 - index)
                if nearest < ramp:
                    step_delay = ramp_delays[nearest]
                    step_pwm = ramp_pwms[nearest]
                else:
                    step_delay = middle_delay
                    step_pwm = middle_pwm

                # We set pwm first because step uses the current parameters including the newly set pwm_scale
                # step() continues from where the last movement ended
                if index < left_steps:
                    self.left_motor.pwm_scale = step_pwm
                    self.left_motor.step(left_step)
                if index < right_steps:
                    self.right_motor.pwm_scale = step_pwm
                    self.right_motor.step(right_step)

                next_time = ticks_add(next_time, step_delay)
                wait = ticks_diff(next_time, ticks_us())
                if wait > 0:
                    sleep_us(wait)
                else:
                    next_time = ticks_us()  # running late: don't try to catch up
        finally:
            self.stop()
            self.left_motor.pwm_scale = left_scale
            self.right_motor.pwm_scale = right_scale

    def _steps_for_distance(self, distance_cm):
        return round(distance_cm / self.distance_per_step)

    def _check_degrees(self, degrees):
        if not isinstance(degrees, int) or not 0 <= degrees <= 359:
            raise ValueError("Degrees must be a whole number from 0 to 359.")

    def drive_cm(
        self,
        distance_cm,
        direction="forward",
        speed_cm_s=None,
        start_speed_cm_s=None,
        acceleration_cm_s2=None,
    ):
        if distance_cm < 0:
            raise ValueError("Distance must be non-negative.")
        if direction not in ["forward", "backward"]:
            raise ValueError("Direction must be 'forward' or 'backward'.")
        direction_step = 1 if direction == "forward" else -1
        steps = self._steps_for_distance(distance_cm)
        self._run_steps(
            steps,
            steps,
            direction_step,
            direction_step,
            speed_cm_s,
            start_speed_cm_s,
            acceleration_cm_s2,
        )

    def turn_in_place(
        self,
        degrees,
        direction="left",
        speed_cm_s=None,
        start_speed_cm_s=None,
        acceleration_cm_s2=None,
    ):
        # Both wheels drive on a circle with the wheelbase as diameter
        self._check_degrees(degrees)
        turning_circumference = math.pi * self.wheelbase_cm
        steps = self._steps_for_distance(turning_circumference * degrees / 360)
        if direction == "left":
            self._run_steps(
                steps, steps, -1, 1, speed_cm_s, start_speed_cm_s, acceleration_cm_s2
            )
        elif direction == "right":
            self._run_steps(
                steps, steps, 1, -1, speed_cm_s, start_speed_cm_s, acceleration_cm_s2
            )
        else:
            raise ValueError("Direction must be 'left' or 'right'.")

    def turn(
        self,
        degrees,
        direction="left",
        speed_cm_s=None,
        start_speed_cm_s=None,
        acceleration_cm_s2=None,
    ):
        # One wheel stands still, the other drives on a circle with the wheelbase as radius
        self._check_degrees(degrees)
        turning_circumference = 2 * math.pi * self.wheelbase_cm
        steps = self._steps_for_distance(turning_circumference * degrees / 360)

        if direction == "left":
            self._run_steps(
                0, steps, 1, 1, speed_cm_s, start_speed_cm_s, acceleration_cm_s2
            )
        elif direction == "right":
            self._run_steps(
                steps, 0, 1, 1, speed_cm_s, start_speed_cm_s, acceleration_cm_s2
            )
        else:
            raise ValueError("Direction must be 'left' or 'right'.")

    def stop(self):
        self.left_motor.stop()
        self.right_motor.stop()
