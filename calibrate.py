from robot_config import robot, sensor

sensor.calibrate(robot)
print("Calibrated. min:", sensor.min_value, "max:", sensor.max_value)
