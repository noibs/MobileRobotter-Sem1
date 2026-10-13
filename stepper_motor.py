# Name: Jacob Jørgensen
# Email: Jajoe26@student.sdu.dk
from machine import Pin, PWM
import time
import math


class StepperMotor:
    def __init__(self, pins, pwm_pct=0.15, frequency=18_000):
        self.pins = []
        self.frequency = frequency
        self.microsteps = None
        self.total_steps = 0
        self.current_step = 0  # position in the active sequence
        self.mode = None  # name of the active mode
        self.pwm_pct = pwm_pct

        for i in pins:
            self.pins.append(PWM(Pin(i, Pin.OUT), freq=self.frequency))

        self.set_step_mode()

    @property
    def pwm_pct(self):
        return self._pwm_pct

    @pwm_pct.setter
    def pwm_pct(self, value):
        # Rebuilds the active sequence with the new PWM, keeps the position
        if not 0 < value <= 1:
            raise ValueError("pwm_pct must be between 0 and 1")
        self._pwm_pct = value
        if self.mode is not None:
            step = self.current_step
            self.set_step_mode(self.mode, self.microsteps)
            self.current_step = step

    def _set_step(self, step):
        # Applies one sequence state to the four outputs
        for pin, duty_cycle in zip(self.pins, step):
            pin.duty_u16(int(duty_cycle * 65535))

    def set_step_mode(self, mode="full_step", microsteps=None):
        # Selects full_step, half_step or micro_step (microsteps = substeps per full step)
        if mode == "full_step":
            self.step_sequence = [
                [self.pwm_pct, self.pwm_pct, 0, 0],  # Step 0
                [0, self.pwm_pct, self.pwm_pct, 0],  # Step 1
                [0, 0, self.pwm_pct, self.pwm_pct],  # Step 2
                [self.pwm_pct, 0, 0, self.pwm_pct],  # Step 3
            ]
        elif mode == "half_step":
            self.step_sequence = [
                [self.pwm_pct, 0, 0, 0],  # Step 0
                [self.pwm_pct, self.pwm_pct, 0, 0],  # Step 1
                [0, self.pwm_pct, 0, 0],  # Step 2
                [0, self.pwm_pct, self.pwm_pct, 0],  # Step 3
                [0, 0, self.pwm_pct, 0],  # Step 4
                [0, 0, self.pwm_pct, self.pwm_pct],  # Step 5
                [0, 0, 0, self.pwm_pct],  # Step 6
                [self.pwm_pct, 0, 0, self.pwm_pct],  # Step 7
            ]
        elif mode == "micro_step":
            self.step_sequence = self.generate_microstep_sequence(microsteps)
        else:
            raise ValueError("Mode not valid, use: full_step/half_step/micro_step")
        self.mode = mode
        self.current_step = 0  # sequence length can change, so start over
        self.microsteps = microsteps

    def generate_microstep_sequence(self, microsteps):
        """Generate and return a complete microstep sequence."""
        if not isinstance(microsteps, int) or microsteps < 1:
            raise ValueError("microsteps must be a positive integer")
        sequence = []
        steps_per_cycle = 4 * microsteps
        for i in range(steps_per_cycle):
            # Phase angle of this state, A follows cos and B follows sin
            angle = 2 * math.pi * i / steps_per_cycle
            a = math.cos(angle) * self.pwm_pct
            b = math.sin(angle) * self.pwm_pct
            # Signed values to [A+, B+, A-, B-]
            sequence.append([max(a, 0), max(b, 0), max(-a, 0), max(-b, 0)])
        return sequence

    def stop(self):
        # All four outputs off
        for i in self.pins:
            i.duty_u16(int(0))

    def move_stepper(self, steps, direction, delay=0.001):
        # Moves a number of steps in the selected sequence and updates total_steps
        if delay < 0.001:
            raise Exception("Delay is too small, must be above 0.001 Sec.")
        if steps < 0:
            raise Exception("Steps can't be negative, use direction: forward/backward")
        try:
            for _ in range(steps):
                if direction == "forward":
                    self._set_step(self.step_sequence[self.current_step])
                    self.total_steps += 1
                    self.current_step += 1
                    if self.current_step == len(self.step_sequence):
                        self.current_step = 0
                    time.sleep(delay)

                elif direction == "backward":
                    self._set_step(self.step_sequence[self.current_step])
                    self.total_steps -= 1
                    self.current_step -= 1
                    if self.current_step < 0:
                        self.current_step = len(self.step_sequence) - 1
                    time.sleep(delay)
                else:
                    raise Exception("Direction not valid, use: forward/backward")
        finally:
            self.stop()

    def run_stepper(self, direction="forward", delay=0.01, interrupt_pin=27):
        # Runs until the pin goes low or Ctrl+C, always stops the outputs
        interrupt = Pin(interrupt_pin, Pin.IN, Pin.PULL_UP)
        try:
            while interrupt.value() != 0:
                self.move_stepper(1, direction, delay)
        finally:
            self.stop()
