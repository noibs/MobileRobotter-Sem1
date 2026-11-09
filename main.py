from stepper_motor import StepperMotor
from differential_drive import DifferentialDrive
import time

test_pwm = 0.2

left = StepperMotor([0, 1, 2, 3], pwm_pct=test_pwm, frequency=20_000, mode="full")
right = StepperMotor([4, 5, 6, 7], pwm_pct=test_pwm, frequency=20_000, mode="full")

robot = DifferentialDrive(left, right, invert_left=False, invert_right=False,)

# Drive in a square
time.sleep(1)
robot.stop()
for _ in range(4):
    robot.drive_cm(300, speed_cm_s=130, start_speed_cm_s=5, acceleration_cm_s2=70)
    time.sleep(1)
    robot.turn_in_place(180, "left", speed_cm_s=80, start_speed_cm_s=8, acceleration_cm_s2=35)
    time.sleep(1)


# robot.drive_cm(100, speed_cm_s=40, start_speed_cm_s=12, acceleration_cm_s2=30)

"""
# With ramp
robot.drive_cm(300, speed_cm_s=50, start_speed_cm_s=12, acceleration_cm_s2=30)
time.sleep(1)

# Without ramp (starts at full speed)
#robot.drive_cm(100, speed_cm_s=34, start_speed_cm_s=34)
"""
