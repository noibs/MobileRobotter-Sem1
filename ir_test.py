import time, sys
from machine import ADC, Pin

from robot_config import robot

adc = ADC(28)

led = Pin(16, Pin.OUT)

led.on()

TURN_AT = 2500     # ms before the turn starts
TOTAL_DEG = 180
STEP_DEG = 1       # take a reading every 5 degrees

sys.stdin.readline()          # wait for "start" from the PC
start = time.ticks_ms()

# Wait until it's time to turn (nothing is logged here)
while time.ticks_diff(time.ticks_ms(), start) < TURN_AT:
    time.sleep_ms(10)

# Turn in small chunks, logging after each one
#

angle = 0
while angle < TOTAL_DEG:
    t = time.ticks_diff(time.ticks_ms(), start)
    print(f"{t},{angle},{adc.read_u16()},{adc.read_u16() < 15_000}")
    robot.turn_in_place(STEP_DEG, "left", speed_cm_s=13.7, start_speed_cm_s=13.7)
    angle += STEP_DEG

for i in range(200):
    t = time.ticks_diff(time.ticks_ms(), start)
    print(f"{t},{angle},{adc.read_u16()},{adc.read_u16() < 15_000}")
    time.sleep_ms(10)
angle = 180
while angle > 0:
    t = time.ticks_diff(time.ticks_ms(), start)
    print(f"{t},{angle},{adc.read_u16()},{adc.read_u16() < 15_000}")
    robot.turn_in_place(STEP_DEG, "right", speed_cm_s=13.7, start_speed_cm_s=13.7)
    angle -= STEP_DEG

# Final reading at the end of the turn
t = time.ticks_diff(time.ticks_ms(), start)
print(f"{t},{angle},{adc.read_u16()},{adc.read_u16() < 15_000}")
print("done")
