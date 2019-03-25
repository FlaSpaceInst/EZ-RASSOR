import time
import Adafruit_PCA9685
import mpu6050
import RPi.GPIO as GPIO


class Wheel:
    """It rotates forwards and backwards!"""
    FORWARD = True
    BACKWARD = False

    def __init__(self,
                 pwm_pin,
                 gpio_pins,
                 speed,
                 driver,
                 gpio_mode=GPIO.BCM,
                 enable_gpio_warnings=False):
        """Initialize a wheel with some GPIO and PWM settings."""
        self.pwm_pin = pwm_pin
        self.gpio_pins = gpio_pins
        self.speed = speed
        self.driver = driver

        GPIO.setmode(gpio_mode)
        GPIO.setwarnings(enable_gpio_warnings)
        for pin in self.gpio_pins:
            GPIO.setup(pin, GPIO.OUT)

    def start(self, direction):
        """Start rotating in the given direction!"""
        if direction == Wheel.FORWARD:
            GPIO.output(self.gpio_pins[0], GPIO.HIGH)
            GPIO.output(self.gpio_pins[1], GPIO.LOW)
        elif direction == Wheel.BACKWARD:
            GPIO.output(self.gpio_pins[0], GPIO.LOW)
            GPIO.output(self.gpio_pins[1], GPIO.HIGH)
        driver.set_pwm(self.pwm_pin, 0, self.speed)

    def stop(self):
        """Stop rotating!"""
        driver.set_pwm(self.pwm_pin, 0, 0)
        GPIO.output(self.gpio_pins[0], GPIO.LOW)
        GPIO.output(self.gpio_pins[1], GPIO.LOW)

driver = Adafruit_PCA9685.PCA9685()
driver.set_pwm_freq(60)
"""
print("front up then down")
driver.set_pwm(15, 0, 325)
time.sleep(1)
driver.set_pwm(15, 0, 675)
time.sleep(1)

print("rear up then down")
driver.set_pwm(12, 0, 700)
time.sleep(1)
driver.set_pwm(12, 0, 325)
time.sleep(1)

sensor = mpu6050.mpu6050(0x68)

while True:
    print(sensor.get_gyro_data())
    time.sleep(.5)
"""

"""
driver = Adafruit_PCA9685.PCA9685()
driver.set_pwm_freq(60)
"""
pins = range(8)
wait = .4

rear_left_pwm = 8
rear_right_pwm = 9
front_left_pwm = 11

front_right_pwm = 10

speed = 4000
brightness = 2000
rear_left_gpio = (23, 27)
rear_right_gpio = (22, 17)
front_left_gpio = (19, 16)
front_right_gpio = (26, 20)


rear_left_wheel = Wheel(rear_left_pwm, rear_left_gpio, speed, driver)
rear_right_wheel = Wheel(rear_right_pwm, rear_right_gpio, speed, driver)
front_left_wheel = Wheel(front_left_pwm, front_left_gpio, speed, driver)
front_right_wheel = Wheel(front_right_pwm, front_right_gpio, speed, driver)
wheels = (rear_left_wheel, rear_right_wheel, front_left_wheel, front_right_wheel)

rot_range = range(300, 800, 4)

try:
    while True:
        for pin in pins:
            driver.set_pwm(pin, 0, brightness)
            time.sleep(wait)
            driver.set_pwm(pin, 0, 0)
            time.sleep(wait/2)

        for rot in rot_range:
            driver.set_pwm(14, 0, rot)
            driver.set_pwm(15, 0, rot)
            time.sleep(.01)

        for wheel in wheels:
            wheel.start(Wheel.FORWARD)
            time.sleep(1)
            wheel.start(Wheel.BACKWARD)
            time.sleep(1)
            wheel.stop()

except KeyboardInterrupt:
    for pin in pins:
        driver.set_pwm(pin, 0, 0)
