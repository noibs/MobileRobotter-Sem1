import _thread
import gc
import math
from array import array
from time import sleep_us, ticks_add, ticks_diff, ticks_us

from micropython import const

# Highest PWM allowed without ramping.
MAX_UNRAMPED_PWM_PCT = 0.2

# Index of each value in the shared array between the two cores.
# Rule: every value has ONE core that writes it (RUNNING: both, but only ever to 0 when stopping).
# A 32-bit write is atomic on the RP2040 and SRAM has no cache, so the other core sees it at once.
RUNNING = const(0)  # 1 while driving, set to 0 by either core to stop
LEFT_DELAY = const(1)  # core 1: us between left steps. Negative = backward, 0 = stand still
RIGHT_DELAY = const(2)  # core 1
LEFT_PWM = const(3)  # core 1: pwm_scale for the left motor, 16384 = 100%
RIGHT_PWM = const(4)  # core 1
LEFT_STEPS = const(5)  # core 0: steps the left wheel has taken (both directions count up)
RIGHT_STEPS = const(6)  # core 0
HEARTBEAT = const(7)  # core 1: counts up every loop, so core 0 can see core 1 is alive
MAX_LATE_US = const(8)  # core 0: the latest a step has been, = the jitter
SLOWEST_LOOP_US = const(9)  # core 1: slowest steering loop
CORE1_DONE = const(10)  # core 1: 1 when the steering thread has finished
SHARED_SIZE = const(11)

# If core 1 stops counting HEARTBEAT for this long, core 0 stops the motors
CORE1_TIMEOUT_US = const(100_000)
# Below this a wheel stands still (and the delay would be too big for the shared array)
MIN_WHEEL_SPEED_CM_S = 0.5


def _approach(value, target, max_change):
    # Move value towards target, but at most max_change.
    # Why: this is the ramp. A stepper loses steps if its speed jumps, so every speed change is limited
    if target > value + max_change:
        return value + max_change
    if target < value - max_change:
        return value - max_change
    return target


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
        acceleration_cm_s2=30,
        min_pwm_pct=0.15,
        max_pwm_pct=0.3,
        wheel_acceleration_cm_s2=50,
        full_pwm_speed_cm_s=50,
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

        # Used by the two-core moves (drive, move)
        self.start_speed_cm_s = start_speed_cm_s
        # PWM follows the wheel's actual speed: min PWM at start speed and below, max PWM only from
        # full_pwm_speed_cm_s and up. A slow wheel never gets max PWM, so the coils don't overheat
        if full_pwm_speed_cm_s <= start_speed_cm_s:
            raise ValueError("full_pwm_speed_cm_s must be above start_speed_cm_s.")
        self.full_pwm_speed_cm_s = full_pwm_speed_cm_s
        # How fast one wheel may change speed, so a stepper at high speed isn't jerked into losing steps
        self._check_acceleration(wheel_acceleration_cm_s2)
        self.wheel_acceleration_cm_s2 = wheel_acceleration_cm_s2
        self._min_delay_us = max(left.min_delay_us, right.min_delay_us)
        self.max_speed_cm_s = self.distance_per_step * 1_000_000 / self._min_delay_us
        # Base speed right now. Kept between moves, so a move with end_speed_cm_s > 0 hands its speed to the next
        self.current_speed_cm_s = 0
        self.max_late_us = 0  # jitter of the last move: the latest any step came
        self.slowest_loop_us = 0  # slowest core 1 loop in the last move
        self._shared = array("i", [0] * SHARED_SIZE)
        self._core1_error = None
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

    def _run_while(self, keep_going, left_direction, right_direction, speed_cm_s=None, start_speed_cm_s=None, acceleration_cm_s2=None, deceleration_cm_s2=None, check_every=10):
        left_step = left_direction * self.left_sign
        right_step = right_direction * self.right_sign

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
        # Braking is harder at high speeds, so stopping takes less distance
        if deceleration_cm_s2 is None:
            deceleration_cm_s2 = acceleration_cm_s2
        else:
            self._check_acceleration(deceleration_cm_s2)

        speed_delay = max(
            speed_delay, self.left_motor.min_delay_us, self.right_motor.min_delay_us
        )

        start_speed_delay = max(start_speed_delay, speed_delay)

        # Full ramp, not cut to half the move because we don't know how long the move is
        ramp = self._ramp_steps_needed(start_speed_delay, speed_delay, acceleration_cm_s2)
        ramp_delays, ramp_pwms = self._ramp_delays(
            start_speed_delay, speed_delay, acceleration_cm_s2, ramp
        )
        # Separate ramp for braking, walked backwards when the robot stops
        brake = self._ramp_steps_needed(start_speed_delay, speed_delay, deceleration_cm_s2)
        brake_delays, brake_pwms = self._ramp_delays(
            start_speed_delay, speed_delay, deceleration_cm_s2, brake
        )
        # Same rule as _run_steps: no ramp means min PWM
        cruise_pwm = self.max_pwm_scale if ramp > 0 else self.min_pwm_scale

        i = 0  # how far up the ramp we are: 0 = start speed, ramp = full speed
        j = 0  # how far up the brake ramp we are, used once stopping
        stopping = False
        count = 0  # steps taken, only used to check the sensor every check_every steps

        gc.collect()
        left_scale, right_scale = self.left_motor.pwm_scale, self.right_motor.pwm_scale

        # several parts of this is the same as _run_steps function
        try:
            next_time = ticks_us()
            while True:
                # check the sensor now and then, until we've decided to stop
                if not stopping and count % check_every == 0 and not keep_going():
                    stopping = True
                    # jump to the point on the brake ramp with the same speed.
                    # speed^2 = start^2 + 2*a*steps, so j = i * acceleration / deceleration
                    j = i * brake // ramp if ramp > 0 else 0
                count += 1

                if stopping:
                    if j == 0:
                        break  # back at start speed: done
                    j -= 1  # walk the brake ramp backwards
                    step_delay = brake_delays[j]
                    step_pwm = brake_pwms[j]
                elif i < ramp:
                    step_delay = ramp_delays[i]
                    step_pwm = ramp_pwms[i]
                    i += 1  # walk the ramp forwards
                else:
                    step_delay = speed_delay  # full speed
                    step_pwm = cruise_pwm

                # step both motors
                self.left_motor.pwm_scale = step_pwm
                self.left_motor.step(left_step)
                self.right_motor.pwm_scale = step_pwm
                self.right_motor.step(right_step)

                # wait
                next_time = ticks_add(next_time, step_delay)
                wait = ticks_diff(next_time, ticks_us())
                if wait > 0:
                    sleep_us(wait)
                else:
                    next_time = ticks_us()
        finally:
            self.stop()
            self.left_motor.pwm_scale = left_scale
            self.right_motor.pwm_scale = right_scale

    def drive_while(self, keep_going, direction="forward", speed_cm_s=None, start_speed_cm_s=None, acceleration_cm_s2=None, deceleration_cm_s2=None, check_every=10):
        if direction not in ["forward", "backward"]:
            raise ValueError("Direction must be 'forward' or 'backward'.")
        d = 1 if direction == "forward" else -1
        self._run_while(keep_going, d, d, speed_cm_s, start_speed_cm_s, acceleration_cm_s2, deceleration_cm_s2, check_every)


    def turn_while(self, keep_going, direction="left", speed_cm_s=None, start_speed_cm_s=None, acceleration_cm_s2=None, deceleration_cm_s2=None, check_every=10):
        if direction == "left":
            self._run_while(keep_going, -1, 1, speed_cm_s, start_speed_cm_s, acceleration_cm_s2, deceleration_cm_s2, check_every)
        elif direction == "right":
            self._run_while(keep_going, 1, -1, speed_cm_s, start_speed_cm_s, acceleration_cm_s2, deceleration_cm_s2, check_every)
        else:
            raise ValueError("Direction must be 'left' or 'right'.")


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

    # ===========================================================================
    # Two-core moves. They use nothing above this line except __init__, _check_acceleration and stop().
    #
    #   robot.drive(...), robot.follow_line(...)     WHAT to do: each one gives move() a steer() function
    #                                                 (follow_line is in robot.py, a subclass of this class)
    #        |
    #   robot.move(steer, speed, distance)            HOW: starts the two cores, waits, cleans up
    #        |
    #        +-- core 1: _core1_loop()   the brain, about every 1 ms:
    #        |       steer() -> (left factor, right factor)
    #        |       ramp the base speed, wheel speed = factor * base speed
    #        |       write each wheel's step delay and PWM into self._shared
    #        |
    #        +-- core 0: _core0_loop()   the stepper, all the time:
    #                step a wheel when its delay has passed since its last step
    #
    # Core 0 never waits for core 1, so a slow core 1 loop can't make a step late.
    # ===========================================================================

    def drive(self, speed_cm_s, distance_cm=None, direction="forward", end_speed_cm_s=0, until=None, acceleration_cm_s2=None):
        # Drive straight: both wheels at the base speed, +1 forward or -1 backward.
        # distance_cm=None: until until() returns True (or Ctrl-C), for driving until something happens.
        # end_speed_cm_s > 0: why: so the next move can continue at that speed without stopping first
        if direction not in ["forward", "backward"]:
            raise ValueError("Direction must be 'forward' or 'backward'.")
        sign = 1 if direction == "forward" else -1
        # Why lambda: move() calls steer() every loop, so it needs a function, not just the two numbers
        self.move(lambda: (sign, sign), speed_cm_s, distance_cm, end_speed_cm_s, until, acceleration_cm_s2)

    def driven_cm(self):
        # How far the middle of the robot has driven in the current move (average of the two wheels).
        # Why from the step counters: a stepper moves exactly one step per pulse, so this is exact
        # without wheel encoders. Used by the line follower's D part
        return (self._shared[LEFT_STEPS] + self._shared[RIGHT_STEPS]) * self.distance_per_step / 2

    def move(self, steer, speed_cm_s, distance_cm=None, end_speed_cm_s=0, until=None, acceleration_cm_s2=None):
        # Runs any move on the two cores. steer() is called on core 1 about every 1 ms and returns
        # (left factor, right factor): wheel speed = factor * base speed, negative = backward.
        # Why factors: every kind of move (straight, line following, turns) is then just a different
        # steer(), and the ramps, braking and stepping are written once, here.
        # drive() steers (1, 1), line following steers from the sensor, a tank turn would steer (-1, 1)
        if not 0 < speed_cm_s <= self.max_speed_cm_s:
            raise ValueError("speed_cm_s must be between 0 and %d." % self.max_speed_cm_s)
        if not 0 <= end_speed_cm_s <= self.max_speed_cm_s:
            raise ValueError("end_speed_cm_s must be between 0 and %d." % self.max_speed_cm_s)
        if distance_cm is not None and distance_cm <= 0:
            raise ValueError("distance_cm must be greater than 0, or None to drive until stopped.")
        if acceleration_cm_s2 is None:
            acceleration_cm_s2 = self.acceleration_cm_s2
        else:
            self._check_acceleration(acceleration_cm_s2)

        # From standstill the steppers can start at start speed without a ramp (slower if the move is slower)
        if self.current_speed_cm_s == 0:
            self.current_speed_cm_s = min(self.start_speed_cm_s, speed_cm_s)
            gc.collect()  # only from standstill: a collection takes a few ms while the wheels would wait

        # Start values, so core 0 can step before core 1 has finished its first loop, if we make a continues movement not from standstill.
        shared = self._shared
        for i in range(SHARED_SIZE):
            shared[i] = 0
        left_factor, right_factor = steer()
        left = self.current_speed_cm_s * left_factor
        right = self.current_speed_cm_s * right_factor
        self._write_wheel(left, LEFT_DELAY, LEFT_PWM)
        self._write_wheel(right, RIGHT_DELAY, RIGHT_PWM)
        shared[RUNNING] = 1
        self._core1_error = None
        _thread.start_new_thread(
            self._core1_loop,
            (steer, speed_cm_s, distance_cm, end_speed_cm_s, until, acceleration_cm_s2, left, right),
        )

        finished = False
        try:
            self._core0_loop()
            finished = True
        finally:
            # Also runs on Ctrl-C or an error: core 1 must stop, or it blocks the next move
            shared[RUNNING] = 0
            start = ticks_us()
            while not shared[CORE1_DONE] and ticks_diff(ticks_us(), start) < 200_000:
                sleep_us(100)
            if not finished or end_speed_cm_s == 0 or self._core1_error is not None or not shared[CORE1_DONE]:
                self.stop()
            self.max_late_us = shared[MAX_LATE_US]
            self.slowest_loop_us = shared[SLOWEST_LOOP_US]
        if not shared[CORE1_DONE]:
            raise RuntimeError("Core 1 did not stop. Reset the Pico.")
        if self._core1_error is not None:
            raise self._core1_error

    def _core1_loop(self, steer, speed_cm_s, distance_cm, end_speed_cm_s, until, acceleration_cm_s2, left, right):
        # Core 1, the brain. Floats are fine here: a garbage collection only pauses this loop
        # for a few ms, while core 0 keeps stepping at the last speeds
        shared = self._shared
        try:
            half_step_cm = self.distance_per_step / 2
            wheel_acceleration = self.wheel_acceleration_cm_s2
            max_speed = self.max_speed_cm_s
            # The base speed brakes down to start speed at most, from there the steppers can stop at once
            end_cm_s = max(end_speed_cm_s, self.start_speed_cm_s)
            base = self.current_speed_cm_s
            stopping = False
            beat = 0
            last_time = ticks_us()

            while shared[RUNNING]:
                left_factor, right_factor = steer()

                now = ticks_us()
                loop_us = ticks_diff(now, last_time)
                last_time = now
                if loop_us > shared[SLOWEST_LOOP_US]:
                    shared[SLOWEST_LOOP_US] = loop_us
                beat = (beat + 1) & 0xFFFF
                shared[HEARTBEAT] = beat  # tells core 0's watchdog that core 1 is alive
                dt = loop_us / 1_000_000

                # 1. Target for the base speed
                if not stopping and until is not None and until():
                    stopping = True  # why not stop at once: a stepper at speed loses steps, so brake down
                if distance_cm is None:
                    target = speed_cm_s
                else:
                    # Distance of the middle of the robot = average of the two wheels.
                    # (Same as driven_cm(), written out here because this loop runs ~1000 times a second)
                    remaining_cm = distance_cm - (shared[LEFT_STEPS] + shared[RIGHT_STEPS]) * half_step_cm
                    if remaining_cm <= 0:
                        break
                    # Why: we must start braking early enough to reach the end speed exactly at the end.
                    # The fastest speed we can still brake from in the distance left: v^2 = v_end^2 + 2*a*s
                    target = min(speed_cm_s, math.sqrt(end_cm_s * end_cm_s + 2 * acceleration_cm_s2 * remaining_cm))
                if stopping:
                    if base <= end_cm_s:
                        break
                    target = end_cm_s

                # 2. Ramp the base speed towards the target. Why dt: the loop time varies (sensor read,
                # garbage collection), so the change per loop is acceleration * time since the last loop
                base = _approach(base, target, acceleration_cm_s2 * dt)
                self.current_speed_cm_s = base  # core 0 only reads this after core 1 is done

                # 3. Each wheel towards factor * base speed. Why a separate wheel limit: steering changes
                # one wheel's speed on top of the base ramp, and a sudden steering jump (line lost, a
                # crossing) must not jerk a stepper into losing steps
                max_change = wheel_acceleration * dt
                left = _approach(left, min(max(base * left_factor, -max_speed), max_speed), max_change)
                right = _approach(right, min(max(base * right_factor, -max_speed), max_speed), max_change)

                # 4. Hand the wheel speeds to core 0
                self._write_wheel(left, LEFT_DELAY, LEFT_PWM)
                self._write_wheel(right, RIGHT_DELAY, RIGHT_PWM)
        except Exception as e:
            self._core1_error = e  # written before CORE1_DONE, so core 0 sees it when it sees DONE
        finally:
            # Whatever happens, core 0 must be told to stop stepping
            shared[RUNNING] = 0
            shared[CORE1_DONE] = 1

    def _write_wheel(self, speed_cm_s, delay_index, pwm_index):
        # Wheel speed -> PWM and microseconds between steps, written into the shared array.
        # PWM is written first, so core 0 never steps at a new speed with the old PWM
        speed = abs(speed_cm_s)
        # PWM from the wheel's own speed: min at start speed and below (also standing still),
        # rising to max at full_pwm_speed_cm_s. A slow wheel never gets max PWM
        fraction = (speed - self.start_speed_cm_s) / (self.full_pwm_speed_cm_s - self.start_speed_cm_s)
        fraction = min(max(fraction, 0), 1)
        self._shared[pwm_index] = int(self.min_pwm_scale + (self.max_pwm_scale - self.min_pwm_scale) * fraction)

        if speed < MIN_WHEEL_SPEED_CM_S:
            delay = 0  # stand still
        else:
            delay = max(int(self.distance_per_step * 1_000_000 / speed), self._min_delay_us)
            if speed_cm_s < 0:
                delay = -delay  # backward
        self._shared[delay_index] = delay

    def _core0_loop(self):
        # Core 0, the stepper: steps each wheel on its own timer until shared[RUNNING] is 0.
        # A wheel steps when its delay has passed since its last step. So a wheel at half speed
        # steps evenly in time between the other wheel's steps.
        # Only small ints here: no floats, no new objects, no locks. So this loop never allocates
        # memory, core 1's garbage collection can't pause it, and it never waits for core 1.
        # Why the left and right code is written out twice instead of one helper function:
        # a function call costs ~10-20 us in MicroPython, and every us here is jitter in the steps
        shared = self._shared
        left, right = self.left_motor, self.right_motor
        left_sign, right_sign = self.left_sign, self.right_sign
        now = ticks_us()
        last_left = last_right = last_beat_time = now
        # Delay used for the previous step. Lateness only counts as jitter when the delay didn't change:
        # when core 1 shortens the delay (speeding up), the step is "late" on purpose
        left_prev = right_prev = 0
        beat = shared[HEARTBEAT]
        while shared[RUNNING]:
            now = ticks_us()

            delay = shared[LEFT_DELAY]
            if delay == 0:
                last_left = now  # standing still: the first step comes one delay after it starts again
            else:
                direction = left_sign
                if delay < 0:
                    delay = -delay
                    direction = -direction
                late = ticks_diff(now, last_left) - delay  # negative = not time yet
                if late >= 0:
                    left.pwm_scale = shared[LEFT_PWM]
                    left.step(direction)
                    shared[LEFT_STEPS] += 1
                    if delay == left_prev and late > shared[MAX_LATE_US]:
                        shared[MAX_LATE_US] = late
                    left_prev = delay
                    if late > delay:
                        last_left = now  # more than a whole step late: don't catch up with a burst of steps
                    else:
                        last_left = ticks_add(last_left, delay)  # keep the rhythm: a late step doesn't delay the next

            delay = shared[RIGHT_DELAY]
            if delay == 0:
                last_right = now
            else:
                direction = right_sign
                if delay < 0:
                    delay = -delay
                    direction = -direction
                late = ticks_diff(now, last_right) - delay
                if late >= 0:
                    right.pwm_scale = shared[RIGHT_PWM]
                    right.step(direction)
                    shared[RIGHT_STEPS] += 1
                    if delay == right_prev and late > shared[MAX_LATE_US]:
                        shared[MAX_LATE_US] = late
                    right_prev = delay
                    if late > delay:
                        last_right = now
                    else:
                        last_right = ticks_add(last_right, delay)

            # Watchdog: if core 1 hangs, don't keep driving blind at the last speed
            if shared[HEARTBEAT] != beat:
                beat = shared[HEARTBEAT]
                last_beat_time = now
            elif ticks_diff(now, last_beat_time) > CORE1_TIMEOUT_US:
                raise RuntimeError("Core 1 stopped steering, motors stopped.")

    def stop(self):
        self.left_motor.stop()
        self.right_motor.stop()
        self.current_speed_cm_s = 0
