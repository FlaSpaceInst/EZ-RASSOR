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


# Change these constants if you want to break this node. :)
FREQUENCY = 60
SHIFT_AMOUNT = 5
REAR_CHANNEL = 12
FORWARD_CHANNEL = 15
SLEEP_DURATION = .012
MASK = 0b000011110000

# These constants are the boundary wavelengths for the servos that control the
# arms. The vertical wavelengths point the arms straight up; ground wavelengths
# bring the arms close to the ground. Note that the forward wavelengths are
# inverted versions of the rear wavelengths. This is due to the forward servo
# being mounted backwards. This will hopefully change after another hardware
# revision.
FORWARD_VERTICAL_WAVELENGTH = 325
FORWARD_GROUND_WAVELENGTH = 675
REAR_VERTICAL_WAVELENGTH = 700
REAR_GROUND_WAVELENGTH = 325

NODE_NAME = "arms_driver"
MOVEMENT_TOGGLES_TOPIC = "/ezrassor/movement_toggles"
MESSAGE_FORMAT = "EZRC ({0}.py): %s.".format(NODE_NAME)


def move_arms(toggle_queue,
              forward_channel,
              rear_channel,
              shift_amount,
              forward_vertical_wavelength,
              forward_ground_wavelength,
              rear_vertical_wavelength,
              rear_ground_wavelength,
              driver_frequency,
              sleep_duration):
    """Move the arms of the EZRC.
    
    The arms are controlled by sending boolean 4-tuples to this function via
    the toggle queue. This function is run as a separate process from the ROS
    subscription code so that both actions (moving the arms and listening to
    the ROS topic) can occur simultaneously. 
    """

    # Create a new driver for the PCA9685 board.
    driver = Adafruit_PCA9685.PCA9685()
    driver.set_pwm_freq(driver_frequency)

    # These movement booleans tell the main function loop whether to raise or
    # lower a particular arm.
    up_rear = False
    down_rear = False
    up_forward = False
    down_forward = False

    # Initialize both arms to be vertical.
    rear_arm_wavelength = rear_vertical_wavelength
    forward_arm_wavelength = forward_vertical_wavelength
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
            if forward_arm_wavelength < forward_vertical_wavelength:
                forward_arm_wavelength = forward_vertical_wavelength
            driver.set_pwm(forward_channel, 0, forward_arm_wavelength)
        if down_forward:
            forward_arm_wavelength += shift_amount
            if forward_arm_wavelength > forward_ground_wavelength:
                forward_arm_wavelength = forward_ground_wavelength
            driver.set_pwm(forward_channel, 0, forward_arm_wavelength)
        if up_rear:
            rear_arm_wavelength += shift_amount
            if rear_arm_wavelength > rear_vertical_wavelength:
                rear_arm_wavelength = rear_vertical_wavelength
            driver.set_pwm(rear_channel, 0, rear_arm_wavelength)
        if down_rear:
            rear_arm_wavelength -= shift_amount
            if rear_arm_wavelength < rear_ground_wavelength:
                rear_arm_wavelength = rear_ground_wavelength
            driver.set_pwm(rear_channel, 0, rear_arm_wavelength)

        # If either arm moved this iteration, sleep for some duration.
        if any((up_forward, down_forward, up_rear, down_rear)):
            time.sleep(sleep_duration)

    # Clean up after the loop is broken. 
    driver.set_pwm(rear_channel, 0, rear_vertical_wavelength)
    driver.set_pwm(forward_channel, 0, forward_vertical_wavelength)


def print_status(toggles, message_format):
    """Print status information based on the provided toggles."""
    up_forward, down_forward, up_rear, down_rear = toggles

    if not any(toggles):
        print message_format % "Stopping both arms"
    else:
        if up_forward:
            print message_format % "Moving forward arm up"
        if down_forward:
            print message_format % "Moving forward arm down"
        if up_rear:
            print message_format % "Moving rear arm up"
        if down_rear:
            print message_format % "Moving rear arm down"


# Main entry point to this node.
try:

    # Create a queue and process to move the arms.
    toggle_queue = multiprocessing.Queue()
    movement_process = multiprocessing.Process(target=move_arms,
                                               args=(toggle_queue,
                                                     FORWARD_CHANNEL,
                                                     REAR_CHANNEL,
                                                     SHIFT_AMOUNT,
                                                     FORWARD_VERTICAL_WAVELENGTH,
                                                     FORWARD_GROUND_WAVELENGTH,
                                                     REAR_VERTICAL_WAVELENGTH,
                                                     REAR_GROUND_WAVELENGTH,
                                                     FREQUENCY,
                                                     SLEEP_DURATION))
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
