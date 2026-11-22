import time
from machine import Pin, PWM, Timer


class SG90:
    FREQ_HZ = 50
    PERIOD_S = 1 / FREQ_HZ  # 20 ms: the servo reads one new position per period
    MIN_PULSE_MS = 0.5  # 0 deg
    MAX_PULSE_MS = 2.5  # MAX_ANGLE
    MAX_ANGLE = 180

    def __init__(self, pin_num, initial_angle=None):
        self.pwm = PWM(Pin(pin_num))
        self.pwm.freq(self.FREQ_HZ)
        self.current_angle = None

        if initial_angle is not None:
            self.set_angle(initial_angle)

    def set_angle(self, angle):
        """Move directly to angle at full servo speed."""
        angle = max(0, min(self.MAX_ANGLE, angle))
        pulse_ms = self.MIN_PULSE_MS + angle / self.MAX_ANGLE * (self.MAX_PULSE_MS - self.MIN_PULSE_MS)
        self.pwm.duty_ns(int(pulse_ms * 1_000_000))
        self.current_angle = angle

    def move_to(self, target_angle, speed_deg_per_sec=None):
        """Move to target_angle at an average speed in deg/s, easing in and out."""
        target_angle = max(0, min(self.MAX_ANGLE, target_angle))

        if not speed_deg_per_sec or self.current_angle is None:
            self.set_angle(target_angle)
            return

        start_angle = self.current_angle
        distance = abs(target_angle - start_angle)
        if distance == 0:
            return

        # The move takes distance / speed seconds, with one step per PWM period
        steps = max(1, int(distance / speed_deg_per_sec / self.PERIOD_S))
        for i in range(1, steps + 1):
            s = i / steps
            s = s * s * (3 - 2 * s)  # smooth start and stop, so the package is not jerked off the magnet
            self.set_angle(start_angle + (target_angle - start_angle) * s)
            time.sleep(self.PERIOD_S)

    def deinit(self):
        """Release PWM signal."""
        self.pwm.deinit()


class Collector:
    MAGNET_PICKUP_ANGLE = 0
    MAGNET_RETRACT_ANGLE = 170
    PUSHER_MIN_ANGLE = 0
    PUSHER_MAX_ANGLE = 100

    MAGNET_SPEED = 240
    PUSHER_SPEED = 45

    MAGNET_PWM_FREQ = 20_000
    MAGNET_PULL_S = 0.3  # full power until the package has jumped up to the poles
    MAGNET_HOLD = 0.4  # duty while carrying: 16 % of the full power (P ~ I^2) keeps the coils cool
    MAGNET_MAX_ON_MS = 3000  # the timer turns the magnet off even if the program stops

    def __init__(self, magnet_pin, magnet_servo_pin, pusher_servo_pin):
        self.magnet_pin = magnet_pin
        self.magnet = PWM(Pin(magnet_pin))
        self.magnet.freq(self.MAGNET_PWM_FREQ)
        self.magnet.duty_u16(0)
        self.magnet_timer = Timer()

        self.magnet_servo = SG90(magnet_servo_pin, initial_angle=self.MAGNET_RETRACT_ANGLE)
        # The pusher rests back, so the stack stands on the floor while the magazine fills
        self.pusher_servo = SG90(pusher_servo_pin, initial_angle=self.PUSHER_MIN_ANGLE)

    def set_magnet(self, level):
        """Magnet power from 0 to 1."""
        self.magnet.duty_u16(int(level * 65535))
        if level > 0:
            self.magnet_timer.init(mode=Timer.ONE_SHOT, period=self.MAGNET_MAX_ON_MS, callback=self._magnet_timeout)
        else:
            self.magnet_timer.deinit()

    def _magnet_timeout(self, _timer):
        self.magnet.duty_u16(0)

    def reset(self):
        """Magnet off and both servos back to their rest angles, where they hold."""
        self.set_magnet(0)
        self.magnet_servo.move_to(self.MAGNET_RETRACT_ANGLE, self.MAGNET_SPEED)
        self.pusher_servo.move_to(self.PUSHER_MIN_ANGLE, self.PUSHER_SPEED * 3.5)

    def pickup_package(self):
        """Drive the magnet out, grab the package and bring it back over the magazine."""
        self.magnet_servo.move_to(self.MAGNET_PICKUP_ANGLE, self.MAGNET_SPEED)
        time.sleep(0.2)
        self.set_magnet(1)
        time.sleep(self.MAGNET_PULL_S)
        self.set_magnet(self.MAGNET_HOLD)

        # The stripper pushes the package off the poles at the end of the stroke, so it can be released right away
        self.magnet_servo.move_to(self.MAGNET_RETRACT_ANGLE, self.MAGNET_SPEED)
        time.sleep(0.2)
        self.set_magnet(0)
        time.sleep(0.5)

    def deliver_package(self):
        """Push the bottom package out slowly, then pull back so the stack drops down."""
        self.pusher_servo.move_to(self.PUSHER_MAX_ANGLE, self.PUSHER_SPEED)
        time.sleep(1.0)
        self.pusher_servo.move_to(self.PUSHER_MIN_ANGLE, self.PUSHER_SPEED * 3.5)
        time.sleep(1.0)

    def deinit(self):
        """Turn off the magnet and release the servos."""
        self.set_magnet(0)
        self.magnet.deinit()
        Pin(self.magnet_pin, Pin.OUT, value=0)
        self.magnet_servo.deinit()
        self.pusher_servo.deinit()
