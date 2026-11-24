from time import sleep

from robot_config import robot

# Measures how evenly the two-core engine steps, without the track.
# max_late_us = the latest any step came compared to its planned time.
# Compare it with the time between steps at that speed (1 m/s = about 1370 us)
for speed in (20, 40, 60, 80, 100):
    robot.drive(speed, 100)
    step_us = robot.distance_per_step * 1_000_000 / speed
    print(speed, "cm/s: jitter", robot.max_late_us, "us of", int(step_us), "us per step,",
          "slowest core 1 loop", robot.slowest_loop_us, "us")
    sleep(1)
