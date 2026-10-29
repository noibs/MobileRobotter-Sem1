import math
from time import sleep_us

from stepper_motor import StepperMotor


class DifferentialDrive:
    def __init__(
        self,
        left,
        right,
        wheel_diameter_cm=8.7,
        wheelbase_cm=23.3,
        invert_left=False,
        invert_right=False,
    ):
        # Geometry is in cm and calculated once here instead of in every method
        self.left_motor = left
        self.right_motor = right
        self.wheel_diameter_cm = wheel_diameter_cm
        self.wheelbase_cm = wheelbase_cm
        self.wheel_circumference_cm = math.pi * wheel_diameter_cm
        self.steps_per_revolution = 200 * len(left.step_sequence) // 4
        self.distance_per_step = self.wheel_circumference_cm / self.steps_per_revolution
        # -1 flips a motor that is mounted mirrored, so "forward" drives the wheel forward
        self.left_sign = -1 if invert_left else 1
        self.right_sign = -1 if invert_right else 1
        self.stop()

    def _ramp_delay(
        self, index, total_steps, start_delay_us, target_delay_us, ramp_steps
    ):
        # Use at most half the move
        ramp = min(ramp_steps, total_steps // 2)

        # How many steps to closest end of the move (start or end)
        nearest = min(index, total_steps - 1 - index)

        # Very short move (1 step): just go slowly
        if ramp == 0:
            return start_delay_us

        # Not near either end: full speed
        if nearest >= ramp:
            return target_delay_us

        # Inside the ramp we increase speed with constant acceleration (math done with help from Claude)
        start_speed = 1_000_000 / start_delay_us  # steps per second
        target_speed = 1_000_000 / target_delay_us
        speed = math.sqrt(
            start_speed**2 + (target_speed**2 - start_speed**2) * nearest / ramp
        )
        return int(1_000_000 / speed)

    def _run_steps(
        self,
        left_steps,
        right_steps,
        left_direction,
        right_direction,
        delay_us,
        start_delay_us=8000,  # we need to test these two values
        ramp_steps=50,
    ):
        if left_steps < 0 or right_steps < 0:
            raise ValueError("Steps must be non-negative.")

        # Directions are 1 (forward) or -1 (backward), flipped if mounted mirrored
        left_step = left_direction * self.left_sign
        right_step = right_direction * self.right_sign

        delay_us = max(
            delay_us, self.left_motor.min_delay_us, self.right_motor.min_delay_us
        )

        start_delay_us = max(start_delay_us, delay_us)

        total_steps = max(left_steps, right_steps)
        try:
            # Runs until the wheel with most steps is done, the other stops when its steps are used
            for index in range(max(left_steps, right_steps)):
                # step() continues from where the last movement ended
                if index < left_steps:
                    self.left_motor.step(left_step)
                if index < right_steps:
                    self.right_motor.step(right_step)
                sleep_us(
                    self._ramp_delay(
                        index, total_steps, start_delay_us, delay_us, ramp_steps
                    )
                )
        finally:
            self.stop()

    def _steps_for_distance(self, distance_cm):
        return round(distance_cm / self.distance_per_step)

    def _check_degrees(self, degrees):
        if not isinstance(degrees, int) or not 0 <= degrees <= 359:
            raise ValueError("Degrees must be a whole number from 0 to 359.")

    def drive_cm(self, distance_cm, direction="forward", delay_us=10_000):
        if distance_cm < 0:
            raise ValueError("Distance must be non-negative.")
        if direction not in ["forward", "backward"]:
            raise ValueError("Direction must be 'forward' or 'backward'.")
        direction_step = 1 if direction == "forward" else -1
        steps = self._steps_for_distance(distance_cm)
        self._run_steps(steps, steps, direction_step, direction_step, delay_us)

    def turn_in_place(self, degrees, direction="left", delay_us=10_000):
        # Both wheels drive on a circle with the wheelbase as diameter
        self._check_degrees(degrees)
        turning_circumference = math.pi * self.wheelbase_cm
        steps = self._steps_for_distance(turning_circumference * degrees / 360)
        if direction == "left":
            self._run_steps(steps, steps, -1, 1, delay_us)
        elif direction == "right":
            self._run_steps(steps, steps, 1, -1, delay_us)
        else:
            raise ValueError("Direction must be 'left' or 'right'.")

    def turn(self, degrees, direction="left", delay_us=10_000):
        # One wheel stands still, the other drives on a circle with the wheelbase as radius
        self._check_degrees(degrees)
        turning_circumference = 2 * math.pi * self.wheelbase_cm
        steps = self._steps_for_distance(turning_circumference * degrees / 360)

        if direction == "left":
            self._run_steps(0, steps, 1, 1, delay_us)
        elif direction == "right":
            self._run_steps(steps, 0, 1, 1, delay_us)
        else:
            raise ValueError("Direction must be 'left' or 'right'.")

    def stop(self):
        self.left_motor.stop()
        self.right_motor.stop()
