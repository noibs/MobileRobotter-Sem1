import time

from machine import ADC

adc = ADC(28)

while True:
    print(adc.read_u16())
    time.sleep(1)
