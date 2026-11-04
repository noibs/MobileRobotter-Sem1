from stepper_motor import StepperMotor
from differential_drive import DifferentialDrive
import time

test_pwm = 0.2

left = StepperMotor([0, 1, 2, 3], pwm_pct=test_pwm, frequency=20_000, mode="half")
right = StepperMotor([4, 5, 6, 7], pwm_pct=test_pwm, frequency=20_000, mode="half")

robot = DifferentialDrive(left, right, wheel_diameter_cm=8.7, wheelbase_cm=23.3, invert_left=False, invert_right=False,)

# Drive in a square
"""robot._run_steps(2000, 2000, 1, 1, 1000)"""

"""print(robot._ramp_delays(4000, 700, 450))
time.sleep(1)
for _ in range(4):
    robot.drive_cm(100, "forward", delay_us=4000)
    time.sleep(1)
    robot.turn_in_place(90, "left", delay_us=700)
    time.sleep(1)
"""
# With ramp
robot.drive_cm(100, speed_cm_s=34, start_speed_cm_s=17, acceleration_cm_s2=14)
time.sleep(1)

# Without ramp (starts at full speed)
robot.drive_cm(100, speed_cm_s=34, start_speed_cm_s=34)
