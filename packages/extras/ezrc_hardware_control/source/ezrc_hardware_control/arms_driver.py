"""A ROS node that moves the arms on the EZRC.

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


# Relevant constants for this node.
NODE_NAME = "arms_driver"
MASK = 0b000011110000
REAR_ARM_CHANNEL = 2
FORWARD_ARM_CHANNEL = 3
SLEEP_DURATION = .1
GROUND_WAVELENGTH = 560
VERTICAL_WAVELENGTH = 250
HALT_MESSAGE = "Stopping both arms"
DEBUGGING_MESSAGES = (
    "Moving forward arm up",
    "Moving forward arm down",
    "Moving rear arm up",
    "Moving rear arm down",
)


def move_arms(toggle_queue):
    """Move the arms of the EZRC.
    
    The arms are controlled by sending boolean 4-tuples to this function via
    the toggle queue. This function is run as a separate process from the ROS
    subscription code so that both actions (moving the arms and listening to
    the ROS topic) can occur simultaneously. 
    """

    # Create a new driver for the PCA9685 board.
    driver = Adafruit_PCA9685.PCA9685()
    driver.set_pwm_freq(constants.DRIVER_FREQUENCY)

    # These movement booleans tell the main function loop whether to raise or
    # lower a particular arm.
    up_rear = False
    down_rear = False
    up_forward = False
    down_forward = False

    # Initialize both arms to be vertical.
    driver.set_pwm(REAR_ARM_CHANNEL, 0, VERTICAL_WAVELENGTH)
    driver.set_pwm(FORWARD_ARM_CHANNEL, 0, VERTICAL_WAVELENGTH)

    while True:

        # Attempt to read some toggles (a 4-tuple) from the queue. If nothing is
        # available then the movement booleans remain unchanged. If the None
        # type is retrieved from the queue, break the loop and let the function
        # end. Otherwise, split the fetched toggles between the 4 movement booleans.
        try:
            toggles = toggle_queue.get(False)
            if toggles == None:
                break
            else:
                up_forward, down_forward, up_rear, down_rear = toggles
        except Queue.Empty:
            pass

        # For each true movement boolean, move the appropriate arm.
        if up_forward:
            driver.set_pwm(FORWARD_ARM_CHANNEL, 0, VERTICAL_WAVELENGTH)
        if down_forward:
            driver.set_pwm(FORWARD_ARM_CHANNEL, 0, GROUND_WAVELENGTH)

        if up_rear:
            driver.set_pwm(REAR_ARM_CHANNEL, 0, VERTICAL_WAVELENGTH)
        if down_rear:
            driver.set_pwm(REAR_ARM_CHANNEL, 0, GROUND_WAVELENGTH)

        time.sleep(SLEEP_DURATION)

    # Clean up after the loop is broken. 
    driver.set_pwm(REAR_ARM_CHANNEL, 0, VERTICAL_WAVELENGTH)
    driver.set_pwm(FORWARD_ARM_CHANNEL, 0, VERTICAL_WAVELENGTH)


def start_node():
    """Initialize this node and start the action!"""
    try:

        # Create a queue and process to move the arms.
        toggle_queue = multiprocessing.Queue()
        movement_process = multiprocessing.Process(
            target=move_arms,
            args=(
                toggle_queue,
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

    # Finally, send a kill message (None) to the movement process and wait for
    # it to die, then exit.
    finally:
        toggle_queue.put(None, False)
        movement_process.join()
