from robot_config import robot

# Start slow and raise the speed step by step (20 -> 40 -> 60 -> 80 -> 100) to find what the motors can do
robot.follow_line(20, 100)
print("Jitter (latest step):", robot.max_late_us, "us")
print("Slowest core 1 loop:", robot.slowest_loop_us, "us")

# Round the track forever, stop with Ctrl-C:
# robot.follow_line(60)

# Package delivery: fast on the long straight, slow the last part to the delivery point.
# end_speed_cm_s=20 means the robot doesn't stop between the two moves
# robot.follow_line(100, 150, end_speed_cm_s=20)
# robot.follow_line(20, 30)
