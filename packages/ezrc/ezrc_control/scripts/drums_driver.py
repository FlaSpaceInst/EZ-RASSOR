#!/usr/bin/env python
"""A ROS node that spins the drums on the EZRC.

Written by Tiger Sachse.
Part of the EZ-RASSOR suite of software.
"""
import time
import rospy
import Queue
import std_msgs
import utilities
import itertools
import multiprocessing
import RPi.GPIO as GPIO


# Change these constants if you want to break this node. :)
SLEEP_DURATION = .2
MASK = 0b000000001111
REAR_PINS = (13, 19, 26)
FORWARD_PINS = (20, 21, 16)

NODE_NAME = "drums_driver"
MOVEMENT_TOGGLES_TOPIC = "/ezrassor/movement_toggles"
MESSAGE_FORMAT = "EZRC ({0}.py): %s.".format(NODE_NAME)


def rotate_drums(nibble_queue,
                 forward_pins,
                 rear_pins,
                 sleep_duration,
                 gpio_mode=GPIO.BCM,
                 enable_gpio_warnings=False):
    """Rotate the drums of the EZRC.
    
    The drums are controlled by sending boolean 4-tuples to this function via
    the nibble queue. This function is run as a separate process from the ROS
    subscription code so that both actions (rotating the drums and listening to
    the ROS topic) can occur simultaneously. 
    """

    # Initialize all required GPIO pins.
    GPIO.setmode(gpio_mode)
    GPIO.setwarnings(enable_gpio_warnings)
    for pin in itertools.chain(forward_pins, rear_pins):
        GPIO.setup(pin, GPIO.OUT)

    # These rotation booleans tell the main function loop whether to rotate a
    # drum in a particular direction or not.
    dig_rear = False
    dump_rear = False
    dig_forward = False
    dump_forward = False

    # These iterators track the current pin that should be lit up to simulate
    # the rotation of a drum. The itertools.cycle iterable makes it easy to
    # loop through a tuple. :)
    rear_iterator = iter(itertools.cycle(rear_pins))
    forward_iterator = iter(itertools.cycle(forward_pins))
    reversed_rear_iterator = iter(itertools.cycle(reversed(rear_pins)))
    reversed_forward_iterator = iter(itertools.cycle(reversed(forward_pins)))

    while True:

        # Attempt to read a nibble (4-tuple) from the queue. If nothing is
        # available then the rotation booleans remain unchanged. If the None
        # type is retrieved from the queue, break the loop and let the function
        # end. Otherwise, split the fetched nibble between the 4 rotation booleans.
        try:
            nibble = nibble_queue.get(False)
            if nibble == None:
                break
            else:
                dig_forward, dump_forward, dig_rear, dump_rear = nibble
        except Queue.Empty:
            pass

        # For each true rotation boolean, turn the appropriate pin on.
        if dig_forward:
            GPIO.output(next(forward_iterator), GPIO.HIGH)
        if dump_forward:
            GPIO.output(next(reversed_forward_iterator), GPIO.HIGH)
        if dig_rear:
            GPIO.output(next(rear_iterator), GPIO.HIGH)
        if dump_rear:
            GPIO.output(next(reversed_rear_iterator), GPIO.HIGH)

        # If any pins were turned on this iteration, sleep for some duration
        # and turn off all pins after waking again.
        if any((dig_forward, dump_forward, dig_rear, dump_rear)):
            time.sleep(sleep_duration)
            utilities.turn_off_pins(forward_pins, rear_pins)
            time.sleep(sleep_duration / 2)

    # Clean up after the loop is broken.
    utilities.turn_off_pins(forward_pins, rear_pins)


def print_status(nibble, message_format):
    """Print status information based on the provided nibble."""
    dig_forward, dump_forward, dig_rear, dump_rear = nibble

    if not any(nibble):
        print message_format % "Stopping both drums"
    else:
        if dig_forward:
            print message_format % "Digging with forward drum"
        if dump_forward:
            print message_format % "Dumping from forward drum"
        if dig_rear:
            print message_format % "Digging with rear drum"
        if dump_rear:
            print message_format % "Dumping from rear drum"


# Main entry point to this node.
try:

    # Create a queue and process to rotate the drums.
    nibble_queue = multiprocessing.Queue()
    rotator_process = multiprocessing.Process(target=rotate_drums,
                                              args=(nibble_queue,
                                                    FORWARD_PINS,
                                                    REAR_PINS,
                                                    SLEEP_DURATION))
    rotator_process.start()

    # Initialize this node as a subscriber.
    rospy.init_node(NODE_NAME)
    rospy.Subscriber(MOVEMENT_TOGGLES_TOPIC,
                     std_msgs.msg.Int16,
                     callback=utilities.enqueue_nibble,
                     callback_args=(nibble_queue,
                                    MASK,
                                    MESSAGE_FORMAT,
                                    print_status))
    rospy.spin()

except rospy.ROSInterruptException:
    pass

# Finally, send a kill message (None) to the rotator process and wait for it to
# die, then exit.
finally:
    nibble_queue.put(None, False)
    rotator_process.join()
