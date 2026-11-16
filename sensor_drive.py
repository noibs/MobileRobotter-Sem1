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



while True:
    print("ADC1:", adc1.read_u16(), "ADC2:", adc2.read_u16())
    if adc1.read_u16() >= 45_000 or adc2.read_u16() >= 45_000:
        robot.drive_cm(5, speed_cm_s=15, start_speed_cm_s=5, acceleration_cm_s2=15)
    else:
        robot.stop()
