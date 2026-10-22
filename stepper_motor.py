from machine import Pin, PWM
from time import sleep
import math


class StepperMotor:
    def __init__(self, pins, pwm_pct=0.15, frequency=18_000):
        self.total_steps = 0
        self.min_delay = 0.0025
        self.pins = [PWM(Pin(pin, Pin.OUT)) for pin in pins]
        self.pwm_pct = pwm_pct
        self.frequency = frequency
        self.wheel_diameter = 0.087
        for pin in self.pins:
            pin.freq(self.frequency)

    def set_step_mode(self, mode, microsteps=None):
        on = self.pwm_pct
        # Select full, half, or generated microstepping.
        if mode == "full":
            self.step_sequence = [
                [on, on, 0, 0],  # Step 0
                [0, on, on, 0],  # Step 1
                [0, 0, on, on],  # Step 2
                [on, 0, 0, on],  # Step 3
            ]
        elif mode == "half":
            self.step_sequence = [
                [on, 0, 0, 0],  # Step 0
                [on, on, 0, 0],  # Step 1
                [0, on, 0, 0],  # Step 2
                [0, on, on, 0],  # Step 3
                [0, 0, on, 0],  # Step 4
                [0, 0, on, on],  # Step 5
                [0, 0, 0, on],  # Step 6
                [on, 0, 0, on],  # Step 7
            ]
        elif mode == "micro" and microsteps is not None:
            self.step_sequence = self.generate_microstep_sequence(microsteps)

        else:
            raise ValueError("Invalid step mode. Choose 'full', 'half', or 'micro'.")

    def generate_microstep_sequence(self, microsteps):
        """Generate and return a complete microstep sequence."""
        if not isinstance(microsteps, int) or microsteps < 1:
            raise ValueError("microsteps must be a positive integer")

        sequence = []
        # Calculate every phase position in one complete electrical cycle.
        for step in range(
            4 * microsteps
        ):  # theta  = 0 to 2*pi for 4 phases --- theta = 2 * pi * i (index) / (4 (phases of motor) * microsteps)
            theta = -2 * math.pi * step / (4 * microsteps)
            # Calculate and scale the phase-A and phase-B amplitudes.
            A = self.pwm_pct * math.sin(theta)
            B = self.pwm_pct * math.cos(theta)
            # Map the signed values to [A+, B+, A-, B-].
            A_pos = max(0, A)
            B_pos = max(0, B)
            A_neg = max(0, -A)
            B_neg = max(0, -B)
            # Append each generated state to sequence.
            sequence.append([A_pos, B_pos, A_neg, B_neg])
        return sequence

    def change_pwm_percentage(self, new_pwm_pct):
        self.pwm_pct = new_pwm_pct
        self.step_sequence = [
            [new_pwm_pct if duty_cycle > 0 else 0 for duty_cycle in step]
            for step in self.step_sequence
        ]

    def set_step(self, step):
        # Apply one step to the selected PWM outputs.
        for pin, duty_cycle in zip(self.pins, step):
            pin.duty_u16(int(duty_cycle * 65535))

    def move_stepper(self, steps, direction="forward", delay=0.01):
        # Move the stepper motor a specified number of steps in the given direction.
        delay = max(delay, self.min_delay)

        if direction not in ["forward", "backward"]:
            raise ValueError("Invalid direction. Choose 'forward' or 'backward'.")
        if steps < 0:
            raise ValueError("Steps must be a non-negative integer.")

        sequence = (
            self.step_sequence
            if direction == "forward"
            else list(reversed(self.step_sequence))
        )

        try:
            for _ in range(abs(steps)):
                for step in sequence:
                    print(f"Setting step: {step}")  # Debugging statement
                    self.set_step(step)
                    self.total_steps += 1 if direction == "forward" else -1
                    print(f"Total steps: {self.total_steps}")  # Debugging statement
                    sleep(delay)
        except KeyboardInterrupt:
            self.stop()

    def continuous_rotation(self, direction="forward", delay=0.01):
        # Continuously rotate the stepper motor in the given direction until interrupted.
        delay = max(delay, self.min_delay)

        if direction not in ["forward", "backward"]:
            raise ValueError("Invalid direction. Choose 'forward' or 'backward'.")

        sequence = (
            self.step_sequence
            if direction == "forward"
            else list(reversed(self.step_sequence))
        )

        try:
            while True:
                for step in sequence:
                    print(f"Setting step: {step}")  # Debugging statement
                    self.set_step(step)
                    self.total_steps += 1 if direction == "forward" else -1
                    print(f"Total steps: {self.total_steps}")  # Debugging statement
                    sleep(delay)
        except KeyboardInterrupt:
            self.stop()

    def move_meters(self, distance_meters, direction="forward", delay=0.01):
        if distance_meters < 0:
            raise ValueError("Distance must be non-negative.")
        if direction not in ["forward", "backward"]:
            raise ValueError("Direction must be 'forward' or 'backward'.")

        steps_per_revolution = 200 * len(self.step_sequence) // 4
        wheel_circumference = math.pi * self.wheel_diameter
        steps = round(distance_meters * steps_per_revolution / wheel_circumference)
        sequence = (
            self.step_sequence
            if direction == "forward"
            else list(reversed(self.step_sequence))
        )
        delay = max(delay, self.min_delay)

        for index in range(steps):
            self.set_step(
                sequence[index % len(sequence)]
            )  # index % len(sequence) keeps the index within the sequences valid range using remainder
            self.total_steps += 1 if direction == "forward" else -1
            sleep(delay)

    def stop(self):
        # Set the PWM duty cycle of all outputs to zero.
        self.set_step([0, 0, 0, 0])
