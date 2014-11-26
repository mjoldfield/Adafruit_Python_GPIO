import Adafruit_GPIO as GPIO
import time

gpio = GPIO.get_platform_gpio()

pin = 338

gpio.setup(pin, GPIO.OUT)

gpio.output(pin, 1)
time.sleep(0.5)
gpio.output(pin, 0)
time.sleep(0.5)


