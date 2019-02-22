#!/usr/bin/env python
"""A ROS node that moves the wheels on the EZRC.

Written by Tiger Sachse.
Part of the EZ-RASSOR suite of software.
"""
import time
import rospy
import Queue
import std_msgs
import utilities
import multiprocessing
import Adafruit_PCA9685
import RPi.GPIO as GPIO


# Change these constants if you want to break this node. :)
SPEED = 2000
GPIO_MODE = GPIO.BCM
DRIVER_FREQUENCY = 60
MASK = 0b111100000000

LEFT_WHEEL_PWM = 5
RIGHT_WHEEL_PWM = 4
LEFT_WHEEL_GPIO = (17, 18)
RIGHT_WHEEL_GPIO = (22, 27)

NODE_NAME = "wheels_driver"
MOVEMENT_TOGGLES_TOPIC = "/ezrassor/movement_toggles"
MESSAGE_FORMAT = "EZRC ({0}.py): %s.".format(NODE_NAME)


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


def rotate_wheels(toggle_queue,
                  left_wheel,
                  right_wheel,
                  driver):
    """Move the wheels of the EZRC.
    
    The wheels are controlled by sending boolean 4-tuples to this function via
    the toggle queue. This function is run as a separate process from the ROS
    subscription code so that both actions (rotating the wheels and listening
    to the ROS topic) can occur simultaneously. 
    """

    # These rotation booleans tell the main function loop what direction the
    # EZRC's wheels should attempt to move.
    left_forward = False
    left_backward = False
    right_forward = False
    right_backward = False

    while True:

        # Attempt to read some toggles (a 4-tuple) from the queue. If nothing is
        # available then the movement booleans remain unchanged. If the None
        # type is retrieved from the queue, break the loop and let the function
        # end. Otherwise, split the fetched toggles between the 4 rotation
        # booleans and give commands to the wheels.
        try:
            toggles = toggle_queue.get(False)
            if toggles == None:
                break
            else:
                left_forward, left_backward, right_forward, right_backward = toggles

                if left_forward:
                    left_wheel.start(Wheel.FORWARD)
                elif left_backward:
                    left_wheel.start(Wheel.BACKWARD)
                else:
                    left_wheel.stop()

                if right_forward:
                    right_wheel.start(Wheel.FORWARD)
                elif right_backward:
                    right_wheel.start(Wheel.BACKWARD)
                else:
                    right_wheel.stop()
        except Queue.Empty:
            pass

    # Clean up and stop the wheels after the loop is broken. 
    left_wheel.stop()
    right_wheel.stop()


def print_status(toggles, message_format):
    """Print status information based on the provided toggles."""
    left_forward, left_backward, right_forward, right_backward = toggles

    if not any(toggles):
        print message_format % "Stopping the car"
    else:
        if left_forward:
            print message_format % "Driving left side forward"
        if left_backward:
            print message_format % "Driving left side backward"
        if right_forward:
            print message_format % "Driving right side forward"
        if right_backward:
            print message_format % "Driving right side backward"


# Main entry point to this node.
try:

    # Create a new driver for the PCA9685 board.
    driver = Adafruit_PCA9685.PCA9685()
    driver.set_pwm_freq(DRIVER_FREQUENCY)

    left_wheel = Wheel(LEFT_WHEEL_PWM, LEFT_WHEEL_GPIO, SPEED, driver)
    right_wheel = Wheel(RIGHT_WHEEL_PWM, RIGHT_WHEEL_GPIO, SPEED, driver)

    # Create a queue and process to rotate the wheels.
    toggle_queue = multiprocessing.Queue()
    movement_process = multiprocessing.Process(target=rotate_wheels,
                                               args=(toggle_queue,
                                                     left_wheel,
                                                     right_wheel,
                                                     driver))
    movement_process.start()

    # Initialize this node as a subscriber.
    rospy.init_node(NODE_NAME)
    rospy.Subscriber(MOVEMENT_TOGGLES_TOPIC,
                     std_msgs.msg.Int16,
                     callback=utilities.enqueue_toggles,
                     callback_args=(toggle_queue,
                                    MASK,
                                    MESSAGE_FORMAT,
                                    print_status))
    rospy.spin()

except rospy.ROSInterruptException:
    pass

# Finally, send a kill message (None) to the movement process and wait for it
# to die, then exit.
finally:
    toggle_queue.put(None, False)
    movement_process.join()
