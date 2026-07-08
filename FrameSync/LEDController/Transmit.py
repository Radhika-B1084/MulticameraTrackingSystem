import RPi.GPIO as GPIO
import socket
import time

LED_PINS = [17, 27, 22, 23]
offsets = [0.0, 0.0, 0.0, 0.0]
kp = 0.8

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

GPIO.setmode(GPIO.BCM)
for pin in LED_PINS:
    GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)

duration = 10000

t4 = (time.monotonic_ns()//1000) - duration

try:
    while True:
        t1 = time.monotonic_ns()//1000
        t1_server = time.time_ns()

        period4 = t1 - t4
        offsets[3] += max(-15, min(15,kp * (duration - period4)))
        print(f"period4: {period4}us  offset[3]: {offsets[3]:.1f}")

        packet =  f"0,{t1},{t1_server},{duration}".encode()
        sock.sendto(packet, ("192.168.0.255", 4210))
        
        GPIO.output(LED_PINS[0], GPIO.HIGH)
        GPIO.output(LED_PINS[3], GPIO.LOW)
        
        sleep_us = duration + offsets[0]
        if sleep_us > 0:
            time.sleep(sleep_us / 1000000)

        t2 = time.monotonic_ns()//1000

        period1 = t2 - t1
        offsets[0] += max(-15, min(15,kp * (duration - period1)))
        print(f"period1: {period1}us  offset[0]: {offsets[0]:.1f}")

        GPIO.output(LED_PINS[1], GPIO.HIGH)
        GPIO.output(LED_PINS[0], GPIO.LOW)
        
        sleep_us = duration + offsets[1]
        if sleep_us > 0:
            time.sleep(sleep_us / 1000000)

        t3 = time.monotonic_ns() //1000
        
        period2 = t3 - t2
        offsets[1] += max(-15, min(15, kp * (duration - period2)))
        print(f"period2: {period2}us  offset[1]: {offsets[1]:.1f}")

        GPIO.output(LED_PINS[2], GPIO.HIGH)
        GPIO.output(LED_PINS[1], GPIO.LOW)
        
        sleep_us = duration + offsets[2]
        if sleep_us > 0:
            time.sleep(sleep_us / 1000000)

        t4 = time.monotonic_ns() //1000

        period3 = t4 - t3
        offsets[2] += max(-15,min(15,kp * (duration - period3)))
        print(f"period3: {period3}us  offset[2]: {offsets[2]:.1f}")

        GPIO.output(LED_PINS[3], GPIO.HIGH)
        GPIO.output(LED_PINS[2], GPIO.LOW)
        
        sleep_us = duration + offsets[3]
        if sleep_us > 0:
            time.sleep(sleep_us / 1000000)
        print("")

except KeyboardInterrupt:
    GPIO.cleanup()
