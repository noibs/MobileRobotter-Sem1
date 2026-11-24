from differential_drive import DifferentialDrive


class Robot(DifferentialDrive):
    # Why a subclass of DifferentialDrive: robot.drive(), robot.drive_cm() and robot.follow_line()
    # all work on one object, while differential_drive.py still knows nothing about lines and sensors

    def __init__(self, left, right, sensor, kp, kd, d_distance_cm, noise_floor, **drive_settings):
        # Why no default values here: two robots need different tuning, so every robot_config.py must
        # choose its own. Forgetting one gives an error instead of the other robot's value
        # Why **drive_settings: every other setting (wheel size, acceleration ...) goes on unchanged
        # to DifferentialDrive, so it doesn't have to be repeated here
        super().__init__(left, right, **drive_settings)
        self.sensor = sensor
        self.kp = kp
        self.kd = kd
        self.d_distance_cm = d_distance_cm
        self.noise_floor = noise_floor
        self._reset_line_memory()

    def follow_line(self, speed_cm_s, distance_cm=None, end_speed_cm_s=0, until=None, acceleration_cm_s2=None):
        # Follow the line. distance_cm=None: forever, until until() returns True (or Ctrl-C).
        # end_speed_cm_s > 0: why: so two moves can be chained without stopping, e.g. fast on the
        # straight and slow the last part to the delivery point:
        #     robot.follow_line(100, 150, end_speed_cm_s=20)
        #     robot.follow_line(20, 30)
        if self.sensor.min_value is None:
            raise ValueError("Line sensor is not calibrated, run calibrate.py.")
        # Why reset: what we remember from the last follow belongs to another place on the track
        self._reset_line_memory()
        # Why self.steer_along_line without (): we hand move() the function itself, and move() calls it
        self.move(self.steer_along_line, speed_cm_s, distance_cm, end_speed_cm_s, until, acceleration_cm_s2)

    def _reset_line_memory(self):
        # What steer_along_line() must remember from one call to the next
        self.last_side = 1.0  # side the line was last seen on: 1 = right, -1 = left
        self.d_start_error = None  # error where the current D measurement started, None = start a new one
        self.d_start_cm = 0.0  # distance driven where the current D measurement started
        self.derivative = 0.0  # last measured change in error per cm

    def steer_along_line(self):
        # move() calls this on core 1 about every 1 ms. It returns how much of the base speed each
        # wheel gets. Why factors and not cm/s: the robot then drives the same curve at every speed
        error = self.sensor.read_position(self.noise_floor)  # -1.0 to 1.0, None = no sensor sees the line

        if error is None:
            # Why: without the line there is no error to steer by. Driving straight would lose it for good,
            # so steer as hard as possible to the side it disappeared on (that's where a sharp curve went)
            error = self.last_side
            # Why no D: the jump to +-1 isn't the robot moving, it would give D a big false kick
            self.d_start_error = None
            self.derivative = 0.0
        else:
            if error != 0:
                self.last_side = 1.0 if error > 0 else -1.0

            # D = change in error / change in distance.
            # Why per cm and not per second: at double speed the error changes twice as fast for the
            # same angle. Per cm, KD means the same at every speed, just like KP
            driven_cm = self.driven_cm()
            if self.d_start_error is None:
                self.d_start_error = error
                self.d_start_cm = driven_cm
            elif driven_cm - self.d_start_cm >= self.d_distance_cm:
                self.derivative = (error - self.d_start_error) / (driven_cm - self.d_start_cm)
                self.d_start_error = error
                self.d_start_cm = driven_cm

        # PD controller. P turns towards the line, D works against the error changing.
        # Why plus and not minus for D: its sign comes from the derivative. On the way in to the line the
        # error shrinks, the derivative is negative, and D takes some of P's turn away = damping
        correction = self.kp * error + self.kd * self.derivative
        # Why +/-: the wheel on the line's side slows down and the other speeds up, so the robot turns
        # towards the line while the middle keeps the base speed
        return 1 + correction, 1 - correction
