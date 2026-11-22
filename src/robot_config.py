from collector import Collector
from differential_drive import DifferentialDrive
from line_sensor import LineSensor
from stepper_motor import StepperMotor

# The robot's hardware, set up in one place. Every script uses it with:
#     from robot_config import robot, sensor, collector
# Python runs this file once on the first import, so all scripts share the same objects.

left = StepperMotor([0, 1, 2, 3], pwm_pct=0.2, frequency=18_000, mode="full")
right = StepperMotor([4, 5, 6, 7], pwm_pct=0.2, frequency=18_000, mode="full")

robot = DifferentialDrive(left, right, wheel_diameter_cm=8.7, wheelbase_cm=23.3, invert_left=False, invert_right=False)

# Mux select pins GP8-10, COM on GP26. Loads calibration.json by itself
sensor = LineSensor(select_pins=(8, 9, 10), com_pin=26)

collector = Collector(magnet_pin=13, magnet_servo_pin=15, pusher_servo_pin=14)
