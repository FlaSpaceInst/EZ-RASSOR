#!/usr/bin/env python
"""A ROS node that moves the drums on the EZRC.

Written by Tiger Sachse and Harrison Black.
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

MASK = 0b000000001111
NODE_NAME = "drums_node"
TOPIC_NAME = "ezmain_topic"
MESSAGE_FORMAT = "EZRC (drums_node.py): %s."

SLEEP_DURATION = .2
GPIO_MODE = GPIO.BCM
REAR_PINS = (13, 19, 26)
FORWARD_PINS = (20, 21, 16)

def rotate_drums(nibble_queue,
                 forward_pins,
                 rear_pins,
                 sleep_duration,
                 message_format):
    dig_rear = False
    dump_rear = False
    dig_forward = False
    dump_forward = False

    forward_iterator = iter(itertools.cycle(forward_pins))
    reversed_forward_iterator = iter(itertools.cycle(reversed(forward_pins)))
    rear_iterator = iter(itertools.cycle(rear_pins))
    reversed_rear_iterator = iter(itertools.cycle(reversed(rear_pins)))

    while True:
        try:
            nibble = nibble_queue.get(False)
            if nibble == None:
                for pin in forward_pins:
                    GPIO.output(pin, GPIO.LOW)

                for pin in rear_pins:
                    GPIO.output(pin, GPIO.LOW)
                break
            else:
                dig_forward, dump_forward, dig_rear, dump_rear = nibble
        except Queue.Empty:
            pass

        if dig_forward:
            print message_format % "Digging with forward drum"
            GPIO.output(next(forward_iterator), GPIO.HIGH)
            
        if dump_forward:
            print message_format % "Dumping from forward drum"
            GPIO.output(next(reversed_forward_iterator), GPIO.HIGH)

        if dig_rear:
            print message_format % "Digging with rear drum"
            GPIO.output(next(rear_iterator), GPIO.HIGH)

        if dump_rear:
            print message_format % "Dumping from rear drum"
            GPIO.output(next(reversed_rear_iterator), GPIO.HIGH)

        if any((dig_forward, dump_forward, dig_rear, dump_rear)):
            time.sleep(sleep_duration)
            for pin in itertools.chain(forward_pins, rear_pins):
                GPIO.output(pin, GPIO.LOW)
            time.sleep(sleep_duration / 2)


def enqueue_nibble(instruction, additional_arguments):
    nibble_queue, mask = additional_arguments

    nibble = utilities.get_nibble(instruction.data, mask)
    nibble_queue.put(nibble, False)


try:
    GPIO.setmode(GPIO_MODE)
    GPIO.setwarnings(False)

    for pin in FORWARD_PINS:
        GPIO.setup(pin, GPIO.OUT)

    for pin in REAR_PINS:
        GPIO.setup(pin, GPIO.OUT)

    nibble_queue = multiprocessing.Queue()
    rotator_process = multiprocessing.Process(target=rotate_drums,
                                              args=(nibble_queue,
                                                    FORWARD_PINS,
                                                    REAR_PINS,
                                                    SLEEP_DURATION,
                                                    MESSAGE_FORMAT))
    rotator_process.start()

    rospy.init_node(NODE_NAME, anonymous=True)
    rospy.Subscriber(TOPIC_NAME,
                     std_msgs.msg.Int16,
                     callback=enqueue_nibble,
                     callback_args=(nibble_queue, MASK))
    rospy.spin()
except rospy.ROSInterruptException:
    pass
finally:
    nibble_queue.put(None, False)
    rotator_process.join()
