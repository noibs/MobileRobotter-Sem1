from stepper_motor import StepperMotor
from time import sleep
import math


class DifferentialDrive:
    def __init__(self, left, right, wheel_diameter_cm=8.7, wheelbase_cm=23.3):
        # Geometry is in cm and calculated once here instead of in every method
        self.left_motor = left
        self.right_motor = right
        self.wheel_diameter_cm = wheel_diameter_cm
        self.wheelbase_cm = wheelbase_cm
        self.wheel_circumference_cm = math.pi * wheel_diameter_cm
        self.steps_per_revolution = 200 * len(left.step_sequence) // 4
        self.distance_per_step = self.wheel_circumference_cm / self.steps_per_revolution

    def _run_steps(
        self, left_steps, right_steps, left_direction, right_direction, delay
    ):
        for direction in (left_direction, right_direction):
            if direction not in ["forward", "backward"]:
                raise ValueError("Directions must be 'forward' or 'backward'.")

        left_sequence = self.left_motor.step_sequence
        right_sequence = self.right_motor.step_sequence
        if left_direction == "backward":
            left_sequence = list(reversed(left_sequence))
        if right_direction == "backward":
            right_sequence = list(reversed(right_sequence))
        # +1 or -1 added to total_steps for every step
        left_count = 1 if left_direction == "forward" else -1
        right_count = 1 if right_direction == "forward" else -1

        delay = max(delay, self.left_motor.min_delay, self.right_motor.min_delay)

        try:
            # Runs until the wheel with most steps is done, the other stops when its steps are used
            for index in range(max(left_steps, right_steps)):
                if index < left_steps:
                    self.left_motor.set_step(left_sequence[index % len(left_sequence)])
                    self.left_motor.total_steps += left_count
                if index < right_steps:
                    self.right_motor.set_step(right_sequence[index % len(right_sequence)])
                    self.right_motor.total_steps += right_count
                sleep(delay)
        except KeyboardInterrupt:
            self.stop()
            raise

    def _steps_for_distance(self, distance_cm):
        return round(distance_cm / self.distance_per_step)

    def drive_cm(self, distance_cm, direction="forward", delay=0.01):
        if distance_cm < 0:
            raise ValueError("Distance must be non-negative.")
        steps = self._steps_for_distance(distance_cm)
        self._run_steps(steps, steps, direction, direction, delay)

    def turn_in_place(self, degrees, direction="left", delay=0.01):
        # Both wheels drive on a circle with the wheelbase as diameter
        turning_circumference = math.pi * self.wheelbase_cm
        steps = self._steps_for_distance(turning_circumference * degrees / 360)

        if direction == "left":
            self._run_steps(steps, steps, "backward", "forward", delay)
        elif direction == "right":
            self._run_steps(steps, steps, "forward", "backward", delay)
        else:
            raise ValueError("Direction must be 'left' or 'right'.")

    def turn(self, degrees, direction="left", delay=0.01):
        # One wheel stands still, the other drives on a circle with the wheelbase as radius
        turning_circumference = 2 * math.pi * self.wheelbase_cm
        steps = self._steps_for_distance(turning_circumference * degrees / 360)

        if direction == "left":
            self._run_steps(0, steps, "forward", "forward", delay)
        elif direction == "right":
            self._run_steps(steps, 0, "forward", "forward", delay)
        else:
            raise ValueError("Direction must be 'left' or 'right'.")

    def stop(self):
        self.left_motor.stop()
        self.right_motor.stop()
