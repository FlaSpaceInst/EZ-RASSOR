"""A ROS node that spins the drums on the EZRC.

Written by Tiger Sachse.
"""
import time
import rospy
import Queue
import std_msgs
import utilities
import itertools
import constants
import multiprocessing
import RPi.GPIO as GPIO


# Relevant constants for this node.
NODE_NAME = "drums_driver"
MASK = 0b000000001111
DRUM_SLEEP_DURATION = .2
REAR_DRUM_PINS = (8, 9, 10, 11)
FORWARD_DRUM_PINS = (12, 13, 14, 15)
HALT_MESSAGE = "Stopping both drums"
DEBUGGING_MESSAGES = (
    "Digging with forward drum",
    "Dumping from forward drum",
    "Digging with rear drum",
    "Dumping from rear drum",
)


def rotate_drums(toggle_queue):
    """Rotate the drums of the EZRC.
    
    The drums are controlled by sending boolean 4-tuples to this function via
    the toggle queue. This function is run as a separate process from the ROS
    subscription code so that both actions (rotating the drums and listening to
    the ROS topic) can occur simultaneously. 
    """

    # Initialize all required GPIO pins.
    GPIO.setmode(constants.GPIO_MODE)
    GPIO.setwarnings(constants.ENABLE_GPIO_WARNINGS)
    pins = itertools.chain(
        FORWARD_DRUM_PINS,
        REAR_DRUM_PINS,
    )
    for pin in pins:
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
    rear_iterator = iter(itertools.cycle(REAR_DRUM_PINS))
    forward_iterator = iter(itertools.cycle(FORWARD_DRUM_PINS))
    reversed_rear_iterator = iter(itertools.cycle(reversed(REAR_DRUM_PINS)))
    reversed_forward_iterator = iter(itertools.cycle(reversed(FORWARD_DRUM_PINS)))

    while True:

        # Attempt to read some toggles (a 4-tuple) from the queue. If nothing is
        # available then the rotation booleans remain unchanged. If the None
        # type is retrieved from the queue, break the loop and let the function
        # end. Otherwise, split the fetched toggles between the 4 rotation booleans.
        try:
            toggles = toggle_queue.get(False)
            if toggles == None:
                break
            else:
                dig_forward, dump_forward, dig_rear, dump_rear = toggles
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
            time.sleep(DRUM_SLEEP_DURATION)
            utilities.turn_off_pins(FORWARD_DRUM_PINS, REAR_DRUM_PINS)
            time.sleep(DRUM_SLEEP_DURATION / 2)

    # Clean up after the loop is broken.
    utilities.turn_off_pins(FORWARD_DRUM_PINS, REAR_DRUM_PINS)


def start_node():
    """Initialize this node and start the fun!"""
    try:

        # Create a queue and process to rotate the drums.
        toggle_queue = multiprocessing.Queue()
        rotator_process = multiprocessing.Process(
            target=rotate_drums,
            args=(
                toggle_queue,
            ),
        )
        rotator_process.start()

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

    # Finally, send a kill message (None) to the rotator process and wait for
    # it to die, then exit.
    finally:
        toggle_queue.put(None, False)
        rotator_process.join()
