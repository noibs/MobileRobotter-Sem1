import time, sys
from machine import ADC, Pin

from differential_drive import DifferentialDrive
from stepper_motor import StepperMotor

test_pwm = 0.2

left = StepperMotor([0, 1, 2, 3], pwm_pct=test_pwm, frequency=18_000, mode="half")
right = StepperMotor([4, 5, 6, 7], pwm_pct=test_pwm, frequency=18_000, mode="half")

robot = DifferentialDrive(left, right, wheel_diameter_cm=8.7, wheelbase_cm=23.3, invert_left=False, invert_right=False)

adc1 = ADC(28)
adc2= ADC(27)
adc3 = ADC(26)
"""
while True:
    print("X-value:", adc1.read_u16())
    print("Y-value:", adc2.read_u16())
    print("Switch-value:", adc3.read_u16())
    time.sleep(1)

"""
while True:

    x_val = adc1.read_u16()
    y_val = adc2.read_u16()
    s_val = adc3.read_u16()

    if x_val > 50_000 and 30_000 < y_val < 40_000:
        print("Moving forwards")
        robot.drive_cm(1, speed_cm_s=15, start_speed_cm_s=5, acceleration_cm_s2=15)
    elif x_val < 20_000 and 30_000 < y_val < 40_000:
        print("Moving backwards")
        robot.drive_cm(1, speed_cm_s=15, start_speed_cm_s=5, acceleration_cm_s2=15, direction="backward")
    elif y_val > 50_000 and 30_000 < x_val < 40_000:
        print("Turning right")
        robot.turn_in_place(5, "right", speed_cm_s=10, start_speed_cm_s=5, acceleration_cm_s2=10)
    elif y_val < 20_000 and 30_000 < x_val < 40_000:
        print("Turning left")
        robot.turn_in_place(5, "left", speed_cm_s=10, start_speed_cm_s=5, acceleration_cm_s2=10)
    elif s_val < 500:
        print("Emergency stop")
        robot.stop()
        break

time.sleep(0.5)
