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
SPEED = 3500
MASK = 0b111100000000
NODE_NAME = "wheels_driver"
REAR_LEFT_WHEEL_CHANNEL = 7
FRONT_LEFT_WHEEL_CHANNEL = 5
REAR_RIGHT_WHEEL_CHANNEL = 6
FRONT_RIGHT_WHEEL_CHANNEL = 4
REAR_LEFT_WHEEL_PINS = (19, 13)
FRONT_LEFT_WHEEL_PINS = (21, 26)
REAR_RIGHT_WHEEL_PINS = (5, 6)
FRONT_RIGHT_WHEEL_PINS = (20, 16)
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
            GPIO.output(self.gpio_pins[1], GPIO.HIGH)
            GPIO.output(self.gpio_pins[0], GPIO.LOW)
        self.driver.set_pwm(self.pwm_pin, 0, SPEED)

    def stop(self):
        """Stop rotating!"""
        self.driver.set_pwm(self.pwm_pin, 0, 0)
        GPIO.output(self.gpio_pins[0], GPIO.LOW)
        GPIO.output(self.gpio_pins[1], GPIO.LOW)


def rotate_wheels(
    toggle_queue,
    rear_left_wheel,
    rear_right_wheel,
    front_left_wheel,
    front_right_wheel):
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
                    rear_left_wheel.start(rear_left_wheel.FORWARD)
                    front_left_wheel.start(front_left_wheel.FORWARD)
                elif left_backward:
                    rear_left_wheel.start(rear_left_wheel.BACKWARD)
                    front_left_wheel.start(front_left_wheel.BACKWARD)
                else:
                    rear_left_wheel.stop()
                    front_left_wheel.stop()

                if right_forward:
                    rear_right_wheel.start(rear_right_wheel.FORWARD)
                    front_right_wheel.start(front_right_wheel.FORWARD)
                elif right_backward:
                    rear_right_wheel.start(rear_right_wheel.BACKWARD)
                    front_right_wheel.start(front_right_wheel.BACKWARD)
                else:
                    rear_right_wheel.stop()
                    front_right_wheel.stop()
        except Queue.Empty:
            pass

    # Clean up and stop the wheels after the loop is broken. 
    rear_left_wheel.stop()
    rear_right_wheel.stop()
    front_left_wheel.stop()
    front_right_wheel.stop()


def start_node():
    """Initialize this node and start the fun!"""
    try:
        driver = Adafruit_PCA9685.PCA9685()
        driver.set_pwm_freq(constants.DRIVER_FREQUENCY)

        rear_left_wheel = Wheel(
            driver,
            REAR_LEFT_WHEEL_CHANNEL,
            REAR_LEFT_WHEEL_PINS,
        )
        rear_right_wheel = Wheel(
            driver,
            REAR_RIGHT_WHEEL_CHANNEL,
            REAR_RIGHT_WHEEL_PINS,
        )
        front_left_wheel = Wheel(
            driver,
            FRONT_LEFT_WHEEL_CHANNEL,
            FRONT_LEFT_WHEEL_PINS,
        )
        front_right_wheel = Wheel(
            driver,
            FRONT_RIGHT_WHEEL_CHANNEL,
            FRONT_RIGHT_WHEEL_PINS,
        )

        # Create a queue and process that rotates the wheels.
        toggle_queue = multiprocessing.Queue()
        movement_process = multiprocessing.Process(
            target=rotate_wheels,
            args=(
                toggle_queue,
                rear_left_wheel,
                rear_right_wheel,
                front_left_wheel,
                front_right_wheel,
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
