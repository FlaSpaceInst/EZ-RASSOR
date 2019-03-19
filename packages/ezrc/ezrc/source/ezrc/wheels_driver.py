"""A ROS node that moves the wheels on the EZRC.

Written by Tiger Sachse.
Part of the EZ-RASSOR suite of software.
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


class Wheel:
    """It rotates forwards and backwards!"""
    FORWARD = True
    BACKWARD = False

    def __init__(self, driver, pwm_pin, gpio_pins):
        """Initialize a new wheel."""
        self.driver = driver
        self.pwm_pin = pwm_pin
        self.gpio_pins = gpio_pins
        self.speed = constants.WHEEL_SPEED

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
        driver.set_pwm(self.pwm_pin, 0, self.speed)

    def stop(self):
        """Stop rotating!"""
        driver.set_pwm(self.pwm_pin, 0, 0)
        GPIO.output(self.gpio_pins[0], GPIO.LOW)
        GPIO.output(self.gpio_pins[1], GPIO.LOW)


def rotate_wheels(toggle_queue, left_wheel, right_wheel):
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


def start_node():
    """"""
    try:

        # Create a new driver for the PCA9685 board.
        driver = Adafruit_PCA9685.PCA9685()
        driver.set_pwm_freq(constants.DRIVER_FREQUENCY)

        left_wheel = Wheel(
            constants.LEFT_WHEEL_CHANNEL,
            constants.LEFT_WHEEL_PINS,
            driver,
        )
        right_wheel = Wheel(
            constants.RIGHT_WHEEL_CHANNEL,
            constants.RIGHT_WHEEL_PINS,
            driver,
        )

        # Create a queue and process to rotate the wheels.
        toggle_queue = multiprocessing.Queue()
        movement_process = multiprocessing.Process(
            target=rotate_wheels,
            args=(
                toggle_queue,
                left_wheel,
                right_wheel,
            ),
        )
        movement_process.start()

        # Initialize this node as a subscriber.
        rospy.init_node(constants.WHEEL_NODE_NAME)
        rospy.Subscriber(
            constants.MOVEMENT_TOGGLES_TOPIC,
            std_msgs.msg.Int16,
            callback=utilities.enqueue_toggles,
            callback_args=(
                toggle_queue,
                constants.WHEEL_MASK,
                constants.MESSAGE_FORMAT.format(
                    constants.WHEEL_NODE_NAME,
                ),
                constants.WHEEL_DEBUGGING_MESSAGES,
                constants.WHEEL_HALT_MESSAGE,
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
