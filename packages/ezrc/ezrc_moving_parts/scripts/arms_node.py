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
SLEEP_DURATION = .025
MASK = 0b000011110000
FREQUENCY = 60
SHIFT_AMOUNT = 5
REAR_CHANNEL = 15
FORWARD_CHANNEL = 12
FORWARD_VERTICAL_WAVELENGTH = 325
FORWARD_GROUND_WAVELENGTH = 675
REAR_VERTICAL_WAVELENGTH = 700
REAR_GROUND_WAVELENGTH = 325

NODE_NAME = "arms_node"
TOPIC_NAME = "ezmain_topic"
MESSAGE_FORMAT = "EZRC ({0}.py): %s.".format(NODE_NAME)


def move_arms(nibble_queue,
              driver,
              forward_channel,
              rear_channel,
              forward_vertical_wavelength,
              forward_ground_wavelength,
              rear_vertical_wavelength,
              rear_ground_wavelength,
              shift_amount,
              sleep_duration):
    """"""
    up_rear = False
    down_rear = False
    up_forward = False
    down_forward = False

    rear_arm_wavelength = rear_vertical_wavelength
    forward_arm_wavelength = forward_vertical_wavelength

    driver.set_pwm(rear_channel, 0, rear_arm_wavelength)
    driver.set_pwm(forward_channel, 0, forward_arm_wavelength)

    while True:
        try:
            nibble = nibble_queue.get(False)
            if nibble == None:
                break
            else:
                up_forward, down_forward, up_rear, down_rear = nibble
        except Queue.Empty:
            pass

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

        if any((up_forward, down_forward, up_rear, down_rear)):
            time.sleep(sleep_duration)


def print_status(nibble, message_format):
    """"""
    up_forward, down_forward, up_rear, down_rear = nibble

    if not any(nibble):
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
    driver = Adafruit_PCA9685.PCA9685()
    driver.set_pwm_freq(FREQUENCY)

    nibble_queue = multiprocessing.Queue()
    movement_process = multiprocessing.Process(target=move_arms,
                                               args=(nibble_queue,
                                                     driver,
                                                     FORWARD_CHANNEL,
                                                     REAR_CHANNEL,
                                                     FORWARD_VERTICAL_WAVELENGTH,
                                                     FORWARd_GROUND_WAVELENGTH,
                                                     REAR_VERTICAL_WAVELENGTH,
                                                     REAR_GROUND_WAVELENGTH,
                                                     SHIFT_AMOUNT,
                                                     SLEEP_DURATION))
    movement_process.start()

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

finally:
    nibble_queue.put(None, False)
    movement_process.join()
