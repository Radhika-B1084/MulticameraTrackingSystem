import time
from arm_iic import ATtinyLED
led1 = ATtinyLED(bus=1)
led2 = ATtinyLED(bus=2)
while True:
    led1.led_control("off")
    led2.led_control("off")
    time.sleep(1)
    led1.led_control("on")
    led2.led_control("on")
    time.sleep(1)
