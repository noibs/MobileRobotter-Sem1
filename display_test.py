from machine import Pin, I2C
import ssd1306


i2c = I2C(0, sda=Pin(8), scl=Pin(9), freq=400_000)
print("I2C devices:", [hex(a) for a in i2c.scan()])   # expect 0x3c or 0x3d

addr = i2c.scan()[0]
oled = ssd1306.SSD1306_I2C(128, 64, i2c, addr=addr)

oled.fill(0)
oled.text("Hello driver!", 0, 15)
oled.text("Speed: 35", 0, 30)
oled.fill_rect(0, 50, 119, 10, 1)
oled.fill_rect(1, 51, 117, 8, 0)
oled.fill_rect(1, 51, 117, 8, 1)
oled.show()