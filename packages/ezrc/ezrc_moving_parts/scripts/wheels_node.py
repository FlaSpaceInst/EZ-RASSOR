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
FRONT_SERVO_PIN = 8
GPIO_MODE = GPIO.BCM
DRIVER_FREQUENCY = 60
MASK = 0b111100000000

LEFT_WHEEL_SPECS = {
    "pwm_pin" : 5,
    "gpio_pins" : (17, 18),
    "servo_frequency" : 450,
}
RIGHT_WHEEL_SPECS = {
    "pwm_pin" : 4,
    "gpio_pins" : (22, 27),
    "servo_frequency" : 325,
}

NODE_NAME = "wheels_node"
TOPIC_NAME = "ezmain_topic"
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
                 servo_frequency,
                 gpio_mode=GPIO.BCM,
                 enable_gpio_warnings=False):
        """Initialize a wheel with some GPIO and PWM settings."""
        self.pwm_pin = pwm_pin
        self.gpio_pins = gpio_pins
        self.speed = speed
        self.driver = driver
        self.servo_frequency = servo_frequency

        GPIO.setmode(gpio_mode)
        GPIO.setwarnings(enable_gpio_warnings)
        for pin in self.gpio_pins:
            GPIO.setup(pin, GPIO.OUT)

    def set_direction(self, direction):
        """Set the direction that this wheel rotates."""
        if direction == Wheel.FORWARD:
            GPIO.output(self.gpio_pins[0], GPIO.HIGH)
            GPIO.output(self.gpio_pins[1], GPIO.LOW)
        elif direction == Wheel.BACKWARD:
            GPIO.output(self.gpio_pins[0], GPIO.LOW)
            GPIO.output(self.gpio_pins[1], GPIO.HIGH)

    def start(self):
        """Start rotating!"""
        driver.set_pwm(self.pwm_pin, 0, self.speed)

    def stop(self):
        """Stop rotating!"""
        driver.set_pwm(self.pwm_pin, 0, 0)
        GPIO.output(self.gpio_pins[0], GPIO.LOW)
        GPIO.output(self.gpio_pins[1], GPIO.LOW)


def rotate_wheels(nibble_queue,
                  left_wheel,
                  right_wheel,
                  front_servo_pin,
                  driver):
    """Move the wheels of the EZRC.
    
    The wheels are controlled by sending boolean 4-tuples to this function via
    the nibble queue. This function is run as a separate process from the ROS
    subscription code so that both actions (rotating the wheels and listening
    to the ROS topic) can occur simultaneously. 
    """

    # These rotation booleans tell the main function loop what direction the
    # EZRC should attempt to move.
    turn_left = False
    turn_right = False
    drive_forward = False
    drive_backward = False

    # Initialize the front servo to point straight forward.
    front_servo_pin_hearth = (left_wheel.servo_frequency
                              + right_wheel.servo_frequency) // 2
    driver.set_pwm(front_servo_pin, 0, front_servo_pin_hearth)

    while True:

        # Attempt to read a nibble (4-tuple) from the queue. If nothing is
        # available then the movement booleans remain unchanged. If the None
        # type is retrieved from the queue, break the loop and let the function
        # end. Otherwise, split the fetched nibble between the 4 rotation
        # booleans and give commands to the wheels.
        try:
            nibble = nibble_queue.get(False)
            if nibble == None:
                break
            else:
                drive_forward, drive_backward, turn_left, turn_right = nibble

                if drive_forward:
                    left_wheel.set_direction(Wheel.FORWARD)
                    right_wheel.set_direction(Wheel.FORWARD)
                    driver.set_pwm(front_servo_pin, 0, front_servo_pin_hearth)
                    left_wheel.start()
                    right_wheel.start()

                elif drive_backward:
                    left_wheel.set_direction(Wheel.BACKWARD)
                    right_wheel.set_direction(Wheel.BACKWARD)
                    driver.set_pwm(front_servo_pin, 0, front_servo_pin_hearth)
                    left_wheel.start()
                    right_wheel.start()

                elif turn_left:
                    left_wheel.set_direction(Wheel.FORWARD)
                    right_wheel.set_direction(Wheel.BACKWARD)
                    driver.set_pwm(front_servo_pin, 0, left_wheel.pwm_pin)
                    left_wheel.start()
                    right_wheel.start()

                elif turn_right:
                    left_wheel.set_direction(Wheel.BACKWARD)
                    right_wheel.set_direction(Wheel.FORWARD)
                    driver.set_pwm(front_servo_pin, 0, right_wheel.pwm_pin)
                    left_wheel.start()
                    right_wheel.start()

                else:
                    driver.set_pwm(front_servo_pin, 0, front_servo_pin_hearth)
                    left_wheel.stop()
                    right_wheel.stop()

        except Queue.Empty:
            pass

    # Clean up and stop the wheels after the loop is broken. 
    driver.set_pwm(front_servo_pin, 0, front_servo_pin_hearth)
    left_wheel.stop()
    right_wheel.stop()


def print_status(nibble, message_format):
    """Print status information based on the provided nibble."""
    drive_forward, drive_backward, turn_left, turn_right = nibble

    if not any(nibble):
        print message_format % "Stopping the car"
    else:
        if drive_forward:
            print message_format % "Driving the car forward"
        if drive_backward:
            print message_format % "Driving the car backward"
        if turn_left:
            print message_format % "Turning the car left"
        if turn_right:
            print message_format % "Turning the car right"


# Main entry point to this node.
try:

    # Create a new driver for the PCA9685 board.
    driver = Adafruit_PCA9685.PCA9685()
    driver.set_pwm_freq(DRIVER_FREQUENCY)

    # Create new Wheel objects using the left and right wheel specs defined at
    # the top of this script.
    left_wheel = Wheel(LEFT_WHEEL_SPECS["pwm_pin"],
                       LEFT_WHEEL_SPECS["gpio_pins"],
                       SPEED,
                       driver,
                       LEFT_WHEEL_SPECS["servo_frequency"])
    
    right_wheel = Wheel(RIGHT_WHEEL_SPECS["pwm_pin"],
                        RIGHT_WHEEL_SPECS["gpio_pins"],
                        SPEED,
                        driver,
                        RIGHT_WHEEL_SPECS["servo_frequency"])

    # Create a queue and process to rotate the wheels.
    nibble_queue = multiprocessing.Queue()
    movement_process = multiprocessing.Process(target=rotate_wheels,
                                               args=(nibble_queue,
                                                     left_wheel,
                                                     right_wheel,
                                                     FRONT_SERVO_PIN,
                                                     driver))
    movement_process.start()

    # Initialize this node as a subscriber.
    rospy.init_node(NODE_NAME, anonymous=True)
    rospy.Subscriber(TOPIC_NAME,
                     std_msgs.msg.Int16,
                     callback=utilities.enqueue_nibble,
                     callback_args=(nibble_queue,
                                    MASK,
                                    MESSAGE_FORMAT,
                                    print_status))
    rospy.spin()

except rospy.ROSInterruptException:
    pass

# Finally, send a kill message (None) to the movement process and wait for it
# to die, then exit.
finally:
    nibble_queue.put(None, False)
    movement_process.join()
