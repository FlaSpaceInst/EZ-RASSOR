import RPi.GPIO as GPIO
import mpu6050
import Adafruit_PCA9685
import time

forward_lights = (9, 5, 10)
rear_lights = (20, 16, 21)

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

for pin in forward_lights:
    GPIO.setup(pin, GPIO.OUT)

for pin in rear_lights:
    GPIO.setup(pin, GPIO.OUT)

speed = .2
for run in range(100):
    for forward_pin, rear_pin in zip(forward_lights, rear_lights):
        GPIO.output(forward_pin, GPIO.HIGH)
        GPIO.output(rear_pin, GPIO.HIGH)
        time.sleep(speed)
        GPIO.output(forward_pin, GPIO.LOW)
        GPIO.output(rear_pin, GPIO.LOW)
        time.sleep(speed/2)
