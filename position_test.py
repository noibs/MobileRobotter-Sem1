from time import sleep_ms

from robot_config import sensor

# Prints the line position and how many sensors see the line. Move the line from left to right
# under the robot: it should go from -1.0 to 1.0.
# Goes 1.0 to -1.0: the sensors are reversed, turn positions around in robot_config.py.
# Jumps back and forth: the mux channels are wired in another order, change positions to match.
# Correct here, but the robot steers away: the left and right motor are swapped in robot_config.py.
# Also try the red start line and a T: how many sensors see them?
while True:
    print(sensor.read_position(), sensor.line_count)
    sleep_ms(500)
