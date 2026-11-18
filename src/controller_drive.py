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

def move_forward():
    return adc1.read_u16() > 50_000 and 0 < adc2.read_u16() < 65535


def move_backward():
    return adc1.read_u16() < 20_000 and 0 < adc2.read_u16() < 65535


def turn_right():
    return adc2.read_u16() > 50_000 and 10_000 < adc1.read_u16() < 65535


def turn_left():
    return adc2.read_u16() < 20_000 and 10_000 < adc1.read_u16() < 65535


def emergency_stop():
    return  adc3.read_u16() < 400

"""
while True:
    print("\n\n\n\n\n\n\n\n\n\n\n\n\n\n")
    print("X:", adc2.read_u16(), "Y:", adc1.read_u16(), "E:", adc3.read_u16())
    time.sleep(0.2)
"""

while True:
    if move_forward():
        robot.drive_while(move_forward, speed_cm_s=70, start_speed_cm_s=10,acceleration_cm_s2=20, deceleration_cm_s2=60, check_every=2)
    elif move_backward():
        robot.drive_while(move_backward, direction="backward", speed_cm_s=30, start_speed_cm_s=5, acceleration_cm_s2=15,deceleration_cm_s2=60, check_every=5)
    elif turn_right():
        robot.turn_while(turn_right, "right", speed_cm_s=10, start_speed_cm_s=5, deceleration_cm_s2=60, check_every=2)
    elif turn_left():
        robot.turn_while(turn_left, "left", speed_cm_s=10, start_speed_cm_s=5, deceleration_cm_s2=60, check_every=2)
    elif emergency_stop():
        robot.stop()
        break
