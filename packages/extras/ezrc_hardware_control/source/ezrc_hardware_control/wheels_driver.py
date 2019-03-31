"""A ROS node that moves the wheels on the EZRC.

Written by Tiger Sachse.
"""
import time
import rospy
import Queue
import std_msgs
import utilities
import constants
import multiprocessing
import Adafruit_PCA9685
import RPi.GPIO as GPIO


# Relevant constants for this node.
NODE_NAME = "wheels_driver"
MASK = 0b111100000000
SPEED = 2000
LEFT_REAR_WHEEL_CHANNEL = 4
RIGHT_REAR_WHEEL_CHANNEL = 5
LEFT_FRONT_WHEEL_CHANNEL = 6
RIGHT_FRONT_WHEEL_CHANNEL = 7
LEFT_REAR_WHEEL_PINS = (16, 20)
RIGHT_REAR_WHEEL_PINS = (26, 21)
LEFT_FRONT_WHEEL_PINS = (5, 6)
RIGHT_FRONT_WHEEL_PINS = (13, 19)
HALT_MESSAGE = "Stopping the wheels"
DEBUGGING_MESSAGES = (
    "Driving left side forward",
    "Driving left side backward",
    "Driving right side forward",
    "Driving right side backward",
)


class Wheel:
    """It rotates forwards and backwards!"""
    FORWARD = True
    BACKWARD = False

    def __init__(self, driver, pwm_pin, gpio_pins):
        """Initialize a new wheel."""
        self.driver = driver
        self.pwm_pin = pwm_pin
        self.gpio_pins = gpio_pins

        GPIO.setmode(constants.GPIO_MODE)
        GPIO.setwarnings(constants.ENABLE_GPIO_WARNINGS)
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
        self.driver.set_pwm(self.pwm_pin, 0, SPEED)

    def stop(self):
        """Stop rotating!"""
        self.driver.set_pwm(self.pwm_pin, 0, 0)
        GPIO.output(self.gpio_pins[0], GPIO.LOW)
        GPIO.output(self.gpio_pins[1], GPIO.LOW)


def rotate_wheels(
    toggle_queue,
    left_front_wheel,
    right_front_wheel,
    left_rear_wheel,
    right_rear_wheel):
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
                    left_front_wheel.start(left_front_wheel.FORWARD)
                    left_rear_wheel.start(left_rear_wheel.FORWARD)
                elif left_backward:
                    left_front_wheel.start(left_front_wheel.BACKWARD)
                    left_rear_wheel.start(left_rear_wheel.BACKWARD)
                else:
                    left_front_wheel.stop()
                    left_rear_wheel.stop()

                if right_forward:
                    right_front_wheel.start(right_front_wheel.FORWARD)
                    right_rear_wheel.start(right_rear_wheel.FORWARD)
                elif right_backward:
                    right_front_wheel.start(right_front_wheel.BACKWARD)
                    right_rear_wheel.start(right_rear_wheel.BACKWARD)
                else:
                    right_front_wheel.stop()
                    right_rear_wheel.stop()
        except Queue.Empty:
            pass

    # Clean up and stop the wheels after the loop is broken. 
    left_front_wheel.stop()
    right_front_wheel.stop()
    left_rear_wheel.stop()
    right_rear_wheel.stop()


def start_node():
    """Initialize this node and start the fun!"""
    try:
        driver = Adafruit_PCA9685.PCA9685()
        driver.set_pwm_freq(constants.DRIVER_FREQUENCY)

        left_front_wheel = Wheel(
            driver,
            LEFT_FRONT_WHEEL_CHANNEL,
            LEFT_FRONT_WHEEL_PINS,
        )
        right_front_wheel = Wheel(
            driver,
            RIGHT_FRONT_WHEEL_CHANNEL,
            RIGHT_FRONT_WHEEL_PINS,
        )
        left_rear_wheel = Wheel(
            driver,
            LEFT_REAR_WHEEL_CHANNEL,
            LEFT_REAR_WHEEL_PINS,
        )
        right_rear_wheel = Wheel(
            driver,
            RIGHT_REAR_WHEEL_CHANNEL,
            RIGHT_REAR_WHEEL_PINS,
        )

        # Create a queue and process that rotates the wheels.
        toggle_queue = multiprocessing.Queue()
        movement_process = multiprocessing.Process(
            target=rotate_wheels,
            args=(
                toggle_queue,
                left_front_wheel,
                right_front_wheel,
                left_rear_wheel,
                right_rear_wheel,
            ),
        )
        movement_process.start()

        # Initialize this node as a subscriber.
        rospy.init_node(NODE_NAME)
        rospy.Subscriber(
            constants.MOVEMENT_TOGGLES_TOPIC,
            std_msgs.msg.Int16,
            callback=utilities.enqueue_toggles,
            callback_args=(
                toggle_queue,
                MASK,
                constants.MESSAGE_FORMAT.format(
                    NODE_NAME,
                    "{0}",
                ),
                DEBUGGING_MESSAGES,
                HALT_MESSAGE,
            ),
        )
        rospy.spin()

    except rospy.ROSInterruptException:
        pass

    # Finally, send a kill message (None) to the movement process and wait for it
    # to die, then exit.
    finally:
        toggle_queue.put(None, False)
        movement_process.join()
