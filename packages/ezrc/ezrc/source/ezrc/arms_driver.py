"""A ROS node that moves the arms on the EZRC.

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
    rear_arm_wavelength = constants.REAR_ARM_VERTICAL_WAVELENGTH
    forward_arm_wavelength = constants.FORWARD_ARM_VERTICAL_WAVELENGTH
    driver.set_pwm(rear_channel, 0, rear_arm_wavelength)
    driver.set_pwm(forward_channel, 0, forward_arm_wavelength)

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

        # For each true movement boolean, move the appropriate arm by the shift
        # amount. If the arm moves past the wavelength boundaries, hold it
        # steady at the boundary. Note that the apparent invertedness of the
        # forward servo is clear in the code below.
        if up_forward:
            forward_arm_wavelength -= shift_amount
            if forward_arm_wavelength < constants.FORWARD_ARM_VERTICAL_WAVELENGTH:
                forward_arm_wavelength = constants.FORWARD_ARM_VERTICAL_WAVELENGTH
            driver.set_pwm(forward_channel, 0, forward_arm_wavelength)
        if down_forward:
            forward_arm_wavelength += shift_amount
            if forward_arm_wavelength > constants.FORWARD_ARM_GROUND_WAVELENGTH:
                forward_arm_wavelength = constants.FORWARD_ARM_GROUND_WAVELENGTH
            driver.set_pwm(forward_channel, 0, forward_arm_wavelength)
        if up_rear:
            rear_arm_wavelength += shift_amount
            if rear_arm_wavelength > constants.REAR_ARM_VERTICAL_WAVELENGTH:
                rear_arm_wavelength = constants.REAR_ARM_VERTICAL_WAVELENGTH
            driver.set_pwm(rear_channel, 0, rear_arm_wavelength)
        if down_rear:
            rear_arm_wavelength -= shift_amount
            if rear_arm_wavelength < constants.REAR_ARM_GROUND_WAVELENGTH:
                rear_arm_wavelength = constants.REAR_ARM_GROUND_WAVELENGTH
            driver.set_pwm(rear_channel, 0, rear_arm_wavelength)

        # If either arm moved this iteration, sleep for some duration.
        if any((up_forward, down_forward, up_rear, down_rear)):
            time.sleep(constants.ARM_SLEEP_DURATION)

    # Clean up after the loop is broken. 
    driver.set_pwm(rear_channel, 0, constants.REAR_ARM_VERTICAL_WAVELENGTH)
    driver.set_pwm(forward_channel, 0, constants.FORWARD_ARM_VERTICAL_WAVELENGTH)


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
        rospy.init_node(constants.ARM_NODE_NAME)
        rospy.Subscriber(
            constants.MOVEMENT_TOGGLES_TOPIC,
            std_msgs.msg.Int16,
            callback=utilities.enqueue_toggles,
            callback_args=(
                toggle_queue,
                constants.ARM_MASK,
                constants.MESSAGE_FORMAT.format(
                    constants.ARM_NODE_NAME,
                ),
                constants.ARM_DEBUGGING_MESSAGES,
                constants.ARM_HALT_MESSAGE,
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
