from stepper_motor import StepperMotor
from differential_drive import DifferentialDrive
import time

left = StepperMotor([0, 1, 2, 3], pwm_pct=0.8, frequency=18_000, mode="half")
right = StepperMotor([4, 5, 6, 7], pwm_pct=0.8, frequency=18_000, mode="half")

robot = DifferentialDrive(left, right, wheel_diameter_cm=8.7, wheelbase_cm=23.3, invert_left=False, invert_right=False,)

time.sleep(5)
for _ in range(4):
    robot.drive_cm(100, "forward", delay_us=5000)
    time.sleep(1)
    robot.turn_in_place(90, "left", delay_us=1000)
    time.sleep(1)
