import time
from machine import ADC, I2C, Pin
import ssd1306 # we install this on the pico

from differential_drive import DifferentialDrive
from stepper_motor import StepperMotor

test_pwm = 0.2

left = StepperMotor([0, 1, 2, 3], pwm_pct=test_pwm, frequency=18_000, mode="half")
right = StepperMotor([4, 5, 6, 7], pwm_pct=test_pwm, frequency=18_000, mode="half")

robot = DifferentialDrive(left, right, wheel_diameter_cm=8.7, wheelbase_cm=23.3, invert_left=False, invert_right=False)

adc1 = ADC(28)
adc2= ADC(27)

emergency = Pin(22, Pin.IN, Pin.PULL_UP)
ESTOP_PRESSED = 0

# Gets center values of joystick on boot up, don't touch it while enabling
CENTER = sum(adc1.read_u16() for _ in range(20)) // 20

# Instantiate the i2c port that controls the oled
i2c = I2C(0, sda=Pin(8), scl=Pin(9), freq=400_000)

addr = i2c.scan()[0]
oled = ssd1306.SSD1306_I2C(128, 64, i2c, addr=addr)


def move_forward():
    return 60_000 > adc1.read_u16() > 34_000 and 0 < adc2.read_u16() < 65535

def move_forward_fast():
    return adc1.read_u16() > 50_000 and 0 < adc2.read_u16() < 65535

def move_backward():
    return   25_000 < adc1.read_u16() < 30_000 and 0 < adc2.read_u16() < 65535

def move_backward_fast():
    return adc1.read_u16() < 25_000 and 0 < adc2.read_u16() < 65535

def turn_right():
    return adc2.read_u16() > 50_000 and 10_000 < adc1.read_u16() < 65535


def turn_left():
    return adc2.read_u16() < 20_000 and 10_000 < adc1.read_u16() < 65535

# Evaluate percentage of stick position.
def stick_percent(adc, center=CENTER):
    if adc >= center:
        p = (adc - center) / (65535 - center)
    else:
        p = (center - adc) / center
    return max(0.0, min(1.0, p))   # clamp, just in case

def update_display(speed, adc):
    pct = 0 if speed == 0 else stick_percent(adc)
    oled.fill(0)
    oled.text(f"Speed: {speed} cm/s", 0, 30)
    oled.rect(0, 50, 119, 10, 1)
    oled.fill_rect(1, 51, int(117 * pct), 8, 1)
    oled.show()

estopped = False

def emergency_stop():
    global estopped
    if not estopped and emergency.value() == ESTOP_PRESSED:
        # Noise filter: must read pressed for ~0.5 ms so interference spike can't trigger it like before
        for _ in range(5):
            time.sleep_us(100)
            if emergency.value() != ESTOP_PRESSED:
                return False
        estopped = True
    return estopped

def unless_estopped(condition):
    # Wraps a stick condition so drive_while also stop when the e-stop is pressed
    return lambda: not emergency_stop() and condition()

while True:
    # checked first, so it wins over any other input
    if emergency_stop():
        robot.stop()
        oled.fill(0)
        oled.text("EMERGENCY STOP!", 8, 28)
        oled.text("Restart machine", 12, 40)
        oled.show()
        break
    elif move_forward():
        update_display(35, adc1.read_u16())
        robot.drive_while(unless_estopped(move_forward), speed_cm_s=35, start_speed_cm_s=20,acceleration_cm_s2=20, deceleration_cm_s2=60, check_every=2)
    elif move_forward_fast():
        update_display(70, adc1.read_u16())
        robot.drive_while(unless_estopped(move_forward_fast), speed_cm_s=70, start_speed_cm_s=20,acceleration_cm_s2=20, deceleration_cm_s2=60, check_every=2)
    elif move_backward():
        update_display(35, adc1.read_u16())
        robot.drive_while(unless_estopped(move_backward), direction="backward", speed_cm_s=35, start_speed_cm_s=5, acceleration_cm_s2=15,deceleration_cm_s2=60, check_every=5)
    elif move_backward_fast():
        update_display(70, adc1.read_u16())
        robot.drive_while(unless_estopped(move_backward_fast), direction="backward", speed_cm_s=70, start_speed_cm_s=20,acceleration_cm_s2=20, deceleration_cm_s2=60, check_every=2)
    elif turn_right():
        update_display(20, adc2.read_u16())
        robot.turn_while(unless_estopped(turn_right), "right", speed_cm_s=20, start_speed_cm_s=20, deceleration_cm_s2=60, check_every=2)
    elif turn_left():
        update_display(20, adc2.read_u16())
        robot.turn_while(unless_estopped(turn_left), "left", speed_cm_s=20, start_speed_cm_s=20, deceleration_cm_s2=60, check_every=2)
    else:
        update_display(0, 0)
