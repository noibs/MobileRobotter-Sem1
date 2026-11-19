from differential_drive import DifferentialDrive
from line_sensor import LineSensor
from stepper_motor import StepperMotor

left = StepperMotor([0, 1, 2, 3], pwm_pct=0.2, frequency=18_000, mode="half")
right = StepperMotor([4, 5, 6, 7], pwm_pct=0.2, frequency=18_000, mode="half")
robot = DifferentialDrive(left, right, wheel_diameter_cm=8.7, wheelbase_cm=23.3)


sensor = LineSensor()
sensor.calibrate(robot)
print("Calibrated. min:", sensor.min_value, "max:", sensor.max_value)
