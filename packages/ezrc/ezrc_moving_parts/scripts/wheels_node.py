#!/usr/bin/env python
"""A ROS node that moves the arms on the EZRC.

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
SPEED = 3000
FREQUENCY = 60
FRONT_SERVO_PIN = 8
MASK = 0b111100000000
LEFT_WHEEL = {
    "pwm" : 5,
    "pins" : (17, 18),
    "frequency" : 400,
}
RIGHT_WHEEL = {
    "pwm" : 4,
    "pins" : (27, 22),
    "frequency" : 550,
}

NODE_NAME = "wheels_node"
TOPIC_NAME = "ezmain_topic"
MESSAGE_FORMAT = "EZRC ({0}.py): %s.".format(NODE_NAME)


def rotate_wheels(nibble_queue,
                  left_wheel,
                  right_wheel,
                  front_servo_pin,
                  speed,
                  driver_frequency,
                  gpio_mode=GPIO.BCM,
                  enable_gpio_warnings=False):
    """Move the arms of the EZRC.
    
    The arms are controlled by sending boolean 4-tuples to this function via
    the nibble queue. This function is run as a separate process from the ROS
    subscription code so that both actions (movement and listening to the ROS
    topic) can occur simultaneously. 
    """

    GPIO.setmode(gpio_mode)
    GPIO.setwarnings(enable_gpio_warnings)

    for pin in left_wheel["pins"]:
        GPIO.setup(pin, GPIO.OUT)
    for pin in right_wheel["pins"]:
        GPIO.setup(pin, GPIO.OUT)

    # Create a new driver for the PCA9685 board.
    driver = Adafruit_PCA9685.PCA9685()
    driver.set_pwm_freq(driver_frequency)

    # These movement booleans tell the main function loop whether to raise or
    # lower a particular arm.
    left_wheel_forward = False
    left_wheel_backward = False
    right_wheel_forward = False
    right_wheel_backward = False

    # Initialize both arms to be vertical.
    front_servo_pin_hearth = (left_wheel["frequency"]
                              + right_wheel["frequency"]) // 2
    driver.set_pwm(front_servo_pin, 0, front_servo_pin_hearth)

    while True:

        # Attempt to read a nibble (4-tuple) from the queue. If nothing is
        # available then the movement booleans remain unchanged. If the None
        # type is retrieved from the queue, break the loop and let the function
        # end. Otherwise, split the fetched nibble between the 4 movement booleans.
        try:
            nibble = nibble_queue.get(False)
            if nibble == None:
                break
            else:
                left_wheel_forward, left_wheel_backward = nibble[:2]
                right_wheel_forward, right_wheel_backward = nibble[2:]

                if left_wheel_forward:
                    GPIO.output(left_wheel["pins"][0], GPIO.HIGH)
                    GPIO.output(left_wheel["pins"][1], GPIO.LOW)
                    driver.set_pwm(front_servo_pin, 0, left_wheel["frequency"])
                    driver.set_pwm(left_wheel["pwm"], 0, speed)

        except Queue.Empty:
            pass

    # Clean up after the loop is broken. 
    driver.set_pwm(front_servo_pin, 0, front_servo_pin_hearth)
    driver.set_pwm(left_wheel["pwm"], 0, 0)
    driver.set_pwm(right_wheel["pwm"], 0, 0)


def print_status(nibble, message_format):
    """Print status information based on the provided nibble."""
    left_wheel_forward, left_wheel_backward = nibble[:2]
    right_wheel_forward, right_wheel_backward = nibble[2:]

    if not any(nibble):
        print message_format % "Stopping both wheels"
    else:
        if left_wheel_forward:
            print message_format % "Rotating left wheel forward"
        if left_wheel_backward:
            print message_format % "Rotating left wheel backward"
        if right_wheel_forward:
            print message_format % "Rotating right wheel forward"
        if right_wheel_backward:
            print message_format % "Rotating right wheel backward"


# Main entry point to this node.
try:

    # Create a queue and process to rotate the wheels.
    nibble_queue = multiprocessing.Queue()
    movement_process = multiprocessing.Process(target=rotate_wheels,
                                               args=(nibble_queue,
                                                     LEFT_WHEEL,
                                                     RIGHT_WHEEL,
                                                     FRONT_SERVO_PIN,
                                                     SPEED,
                                                     FREQUENCY))
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
