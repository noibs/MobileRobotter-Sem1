from machine import Pin, PWM
from time import sleep_us
import math


class StepperMotor:
    def __init__(self, pins, pwm_pct=0.15, frequency=18_000, mode="full", microsteps=None):
        self.total_steps = 0
        self.current_step = 0  # position in step_sequence, kept between movements
        self.min_delay_us = 2500
        self.pins = [PWM(Pin(pin, Pin.OUT)) for pin in pins]
        self.mode = None
        self.pwm_pct = pwm_pct
        self.frequency = frequency
        for pin in self.pins:
            pin.freq(self.frequency)
        self.set_step_mode(mode, microsteps)  # step_sequence exists from the start
        self.stop()
        
    @property
    def pwm_pct(self):
        return self._pwm_pct

    @pwm_pct.setter
    def pwm_pct(self, value):
        if not 0 < value <= 1:
            raise ValueError("pwm_pct must be between 0 and 1")
        self._pwm_pct = value
        if self.mode is not None:
            step = self.current_step
            self.set_step_mode(self.mode, self.microsteps)
            self.current_step = step

    def set_step_mode(self, mode, microsteps=None):
        on = int(self.pwm_pct * 65535)
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
        # Saved so changing pwm rebuilds the same sequence
        self.mode = mode
        self.microsteps = microsteps
        self.current_step = 0

    def generate_microstep_sequence(self, microsteps):
        """Generate and return a complete microstep sequence."""
        if not isinstance(microsteps, int) or microsteps < 1:
            raise ValueError("microsteps must be a positive integer")

        sequence = []
        on = self.pwm_pct * 65535
        # Calculate every phase position in one complete electrical cycle.
        for step in range(
            4 * microsteps
        ):  # theta  = 0 to 2*pi for 4 phases --- theta = 2 * pi * i (index) / (4 (phases of motor) * microsteps)
            theta = -2 * math.pi * step / (4 * microsteps)
            # Calculate and scale the phase-A and phase-B amplitudes.
            A = int(on * math.sin(theta))
            B = int(on * math.cos(theta))
            # Map the signed values to [A+, B+, A-, B-].
            A_pos = max(0, A)
            B_pos = max(0, B)
            A_neg = max(0, -A)
            B_neg = max(0, -B)
            # Append each generated state to sequence.
            sequence.append([A_pos, B_pos, A_neg, B_neg])
        return sequence

    def set_step(self, step):
        # Apply one step to the selected PWM outputs.
        for pin, duty_cycle in zip(self.pins, step):
            pin.duty_u16(duty_cycle)

    def step(self, direction):
        # One step forward (1) or backward (-1) from the current position, wraps around
        self.current_step = (self.current_step + direction) % len(self.step_sequence)
        self.total_steps += direction
        self.set_step(self.step_sequence[self.current_step])

    def move_stepper(self, steps, direction="forward", delay_us=10_000):
        # Move the stepper motor a specified number of steps in the given direction.
        delay_us = max(delay_us, self.min_delay_us)

        if direction not in ["forward", "backward"]:
            raise ValueError("Invalid direction. Choose 'forward' or 'backward'.")
        if steps < 0:
            raise ValueError("Steps must be a non-negative integer.")

        step_direction = 1 if direction == "forward" else -1
        try:
            for _ in range(steps):
                self.step(step_direction)
                sleep_us(delay_us)
        finally:
            self.stop()

    def continuous_rotation(self, direction="forward", delay_us=10_000):
        # Continuously rotate the stepper motor in the given direction until interrupted.
        delay_us = max(delay_us, self.min_delay_us)

        if direction not in ["forward", "backward"]:
            raise ValueError("Invalid direction. Choose 'forward' or 'backward'.")

        step_direction = 1 if direction == "forward" else -1
        try:
            while True:
                self.step(step_direction)
                sleep_us(delay_us)
        finally:
            self.stop()

    def stop(self):
        # Set the PWM duty cycle of all outputs to zero.
        self.set_step([0, 0, 0, 0])
