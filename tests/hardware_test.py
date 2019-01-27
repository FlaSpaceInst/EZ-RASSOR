import RPi.GPIO as GPIO
#import mpu6050
from Adafruit_PCA9685 import PCA9685
import time

# red, white, blue
rear_lights = (13, 19, 26)
forward_lights = (20, 21, 16)

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

for pin in rear_lights:
    GPIO.setup(pin, GPIO.OUT)

for pin in forward_lights:
    GPIO.setup(pin, GPIO.OUT)

speed = .2
for run in range(100):
    for rear_pin, forward_pin in zip(rear_lights, forward_lights):
        GPIO.output(rear_pin, GPIO.HIGH)
        GPIO.output(forward_pin, GPIO.HIGH)
        time.sleep(speed)
        GPIO.output(rear_pin, GPIO.LOW)
        GPIO.output(forward_pin, GPIO.LOW)
        time.sleep(speed/2)

# move the arms up and down
pwm = PCA9685.PCA9685()
pwm.set_pwm_freq(60)
min = 200
max = 700
#channel forward? = 15
#channel rear = 12
pwm.set_pwm(15, 0, 150)
pwm.set_pwm(12, 0, 0)
time.sleep(1)

def rotate_arm(degrees):
    pass

"""
# 11 12 13 15
m0a = 17
m0b = 18
m1a = 27
m1b = 22
pwm = PCA9685()
pwm.set_pwm_freq(60)

GPIO.setup(m0a, GPIO.OUT)
GPIO.setup(m0b, GPIO.OUT)
GPIO.setup(m1a, GPIO.OUT)
GPIO.setup(m1b, GPIO.OUT)
"""

speed = 3000
GPIO.output(m0a, GPIO.LOW)
GPIO.output(m0b, GPIO.HIGH)
pwm.set_pwm(5, 0, speed)


GPIO.output(m1b, GPIO.HIGH)
GPIO.output(m1a, GPIO.LOW)
pwm.set_pwm(4, 0, speed)

time.sleep(4)
pwm.set_pwm(4, 0, 0)
pwm.set_pwm(5, 0, 0)
