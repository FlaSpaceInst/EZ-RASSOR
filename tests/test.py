import time
import Adafruit_PCA9685
import mpu6050


sensor = mpu6050.mpu6050(0x68)

while True:
    print(sensor.get_gyro_data())
    time.sleep(.5)

"""
driver = Adafruit_PCA9685.PCA9685()
driver.set_pwm_freq(60)

pins = range(8)
wait = .4

rot_range = range(300, 800, 4)

try:
    while True:
        for pin in pins:
            driver.set_pwm(pin, 0, 1000)
            time.sleep(wait)
            driver.set_pwm(pin, 0, 0)
            time.sleep(wait/2)

        for rot in rot_range:
            driver.set_pwm(14, 0, rot)
            driver.set_pwm(15, 0, rot)
            time.sleep(.01)
except KeyboardInterrupt:
    for pin in pins:
        driver.set_pwm(pin, 0, 0)
"""
