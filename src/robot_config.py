from collector import Collector
from line_sensor import LineSensor
from robot import Robot
from stepper_motor import StepperMotor

# --- Size of this robot -------------------------------------------------------
WHEEL_DIAMETER_CM = 8.7
WHEELBASE_CM = 23.3

# --- Speed and acceleration ---------------------------------------------------
# A stepper can start at this speed without a ramp
START_SPEED_CM_S = 4
# Used by the old moves (drive_cm, turn ...) when they are not given a speed
DEFAULT_SPEED_CM_S = 15
# How fast the whole robot speeds up and brakes, when a move is not given its own
ACCELERATION_CM_S2 = 30
# How fast one wheel may change speed (two-core moves). Must be above ACCELERATION_CM_S2,
# because steering comes on top of the robot speeding up
WHEEL_ACCELERATION_CM_S2 = 50

# --- Motor current (PWM, 0-1) -------------------------------------------------
# Lowest PWM, and highest PWM allowed
MIN_PWM_PCT = 0.15
MAX_PWM_PCT = 0.3
# Two-core moves: a wheel reaches MAX_PWM_PCT at this speed. Between START_SPEED_CM_S and this it rises evenly
FULL_PWM_SPEED_CM_S = 50

# --- Line following, tuned for this robot -------------------------------------
# P: the further the line is from the middle, the harder we turn back to it.
KP = 1
# D works against the error changing, so the robot eases onto the line.
KD = 0

# D is measured over this distance. Why not every reading: in 1 ms the robot only moves ~0.03 cm, so the
# real change in error is smaller than the sensor noise. Over 1 cm the real change is clearly bigger.
D_DISTANCE_CM = 1.0

# A sensor below this is floor, not line (0 = floor, 1000 = line).
# Why: just above the floor value it is only noise, and 8 noisy sensors would pull the position around
NOISE_FLOOR = 100
# ------------------------------------------------------------------------------

left = StepperMotor([0, 1, 2, 3], pwm_pct=0.2, frequency=18_000, mode="full")
right = StepperMotor([4, 5, 6, 7], pwm_pct=0.2, frequency=18_000, mode="full")

# positions = where mux channel 0-7 sits: negative = left, positive = right.
sensor = LineSensor(
    select_pins=(8, 9, 10), com_pin=26, positions=(7, 5, 3, 1, -1, -3, -5, -7)
)

# Robot = DifferentialDrive + line sensor. Everything after noise_floor goes on to DifferentialDrive
robot = Robot(
    left,
    right,
    sensor,
    kp=KP,
    kd=KD,
    d_distance_cm=D_DISTANCE_CM,
    noise_floor=NOISE_FLOOR,
    wheel_diameter_cm=WHEEL_DIAMETER_CM,
    wheelbase_cm=WHEELBASE_CM,
    invert_left=False,
    invert_right=False,
    speed_cm_s=DEFAULT_SPEED_CM_S,
    start_speed_cm_s=START_SPEED_CM_S,
    acceleration_cm_s2=ACCELERATION_CM_S2,
    wheel_acceleration_cm_s2=WHEEL_ACCELERATION_CM_S2,
    min_pwm_pct=MIN_PWM_PCT,
    max_pwm_pct=MAX_PWM_PCT,
    full_pwm_speed_cm_s=FULL_PWM_SPEED_CM_S,
)

collector = Collector(magnet_pin=13, magnet_servo_pin=15, pusher_servo_pin=14)
