import time

from robot_config import robot

# Drive in a square
time.sleep(1)
robot.stop()
for _ in range(4):
    robot.drive_cm(300, direction="forward", speed_cm_s=130, start_speed_cm_s=5, acceleration_cm_s2=70)
    time.sleep(1)
    robot.turn_in_place(180, "left", speed_cm_s=80, start_speed_cm_s=8, acceleration_cm_s2=35)
    time.sleep(1)
