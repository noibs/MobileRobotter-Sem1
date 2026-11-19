from machine import ADC

from robot_config import robot

adc1 = ADC(28)
adc2= ADC(27)

threshold = 32_000

def both_on_line():
    return adc1.read_u16() >= threshold and adc2.read_u16() >= threshold

def only_adc1_on_line():
    return adc1.read_u16() >= threshold > adc2.read_u16()

def only_adc2_on_line():
    return adc2.read_u16() >= threshold > adc1.read_u16()

while True:
    if both_on_line():
        robot.drive_while(both_on_line, speed_cm_s=30, start_speed_cm_s=5,acceleration_cm_s2=15, deceleration_cm_s2=60, check_every=5)
    elif only_adc1_on_line():
        robot.turn_while(only_adc1_on_line, "right", speed_cm_s=10, start_speed_cm_s=4, check_every=4)
    elif only_adc2_on_line():
        robot.turn_while(only_adc2_on_line, "left", speed_cm_s=10, start_speed_cm_s=4, check_every=4)
    else:
        robot.stop()
        break

"""
while True:
    print("ADC1:", adc1.read_u16(), "ADC2:", adc2.read_u16())
    if adc1.read_u16() >= 40_000 and adc2.read_u16() >= 40_000:
        robot.drive_cm(1, speed_cm_s=15, start_speed_cm_s=15, acceleration_cm_s2=30)
    elif adc1.read_u16() >= 40_000 > adc2.read_u16():
        robot.turn_in_place(1, "right", speed_cm_s=15, start_speed_cm_s=5, acceleration_cm_s2=15)
    elif adc2.read_u16() >= 40_000 > adc1.read_u16():
        robot.turn_in_place(1, "left", speed_cm_s=15, start_speed_cm_s=5, acceleration_cm_s2=15)
    else:
        robot.stop()
"""
