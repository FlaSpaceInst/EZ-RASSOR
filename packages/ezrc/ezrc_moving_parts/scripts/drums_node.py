#!/usr/bin/env python
"""A ROS node that moves the drums on the EZRC.

Written by Tiger Sachse and Harrison Black.
Part of the EZ-RASSOR suite of software.
"""
import time
import rospy
import std_msgs
import utilities
import RPi.GPIO as GPIO

MASK = 0b000000001111
NODE_NAME = "drums_node"
TOPIC_NAME = "ezmain_topic"
MESSAGE_FORMAT = "EZRC (drums_node.py): %s."

SLEEP_DURATION = .2
GPIO_MODE = GPIO.BCM
REAR_PINS = (13, 19, 26)
FORWARD_PINS = (20, 21, 16)

class Drum:
    def __init__(self, pins, sleep_duration):
        self.pins = pins
        self.rotating = False
        self.sleep_duration = sleep_duration

    def is_rotating(self):
        return self.rotating

    def dig(self):
        self.__rotate(self.pins)

    def dump(self):
        self.__rotate(tuple(reversed(self.pins)))
    
    def stop(self):
        self.rotating = False

    def __rotate(self, pins):
        self.rotating = True

        pin_iterator = iter(pins)
        while self.rotating:
            try:
                pin = next(pin_iterator)
            except StopIteration:
                pin_iterator = iter(pins)
                pin = next(pin_iterator) # error here
            finally:
                GPIO.output(pin, GPIO.HIGH)
                time.sleep(self.sleep_duration)
                GPIO.output(pin, GPIO.LOW)
                time.sleep(self.sleep_duration / 2)


def handle_drum_movements(instruction, additional_arguments):
    """Move the drums of the EZRC per the commands encoded in the instruction."""
    forward_drum, rear_drum, mask, message_format = additional_arguments

    nibble = utilities.get_nibble(instruction.data, mask)
    dig_forward, dump_forward, dig_rear, dump_rear = nibble

    if not any((dig_forward, dump_forward, dig_rear, dump_rear)):
        print message_format % "Stopping both drums"
        forward_drum.stop()
        rear_drum.stop()

    else:
        if dig_forward:
            print message_format % "Digging with forward drum"
            forward_drum.dig()
            
        if dump_forward:
            print message_format % "Dumping from forward drum"
            forward_drum.dump()

        if dig_rear:
            print message_format % "Digging with rear drum"
            rear_drum.dig()

        if dump_rear:
            print message_format % "Dumping from rear drum"
            rear_drum.dump()


# Main entry point to the node.
try:
    GPIO.setmode(GPIO_MODE)
    GPIO.setwarnings(False)

    for pin in FORWARD_PINS:
        GPIO.setup(pin, GPIO.OUT)

    for pin in REAR_PINS:
        GPIO.setup(pin, GPIO.OUT)

    rospy.init_node(NODE_NAME, anonymous=True)
    rospy.Subscriber(TOPIC_NAME,
                     std_msgs.msg.Int16,
                     callback=handle_drum_movements,
                     callback_args=(Drum(FORWARD_PINS, SLEEP_DURATION),
                                    Drum(REAR_PINS, SLEEP_DURATION),
                                    MASK,
                                    MESSAGE_FORMAT))
    rospy.spin()
except rospy.ROSInterruptException:
    pass
finally:
    for pin in FORWARD_PINS:
        GPIO.output(pin, GPIO.LOW)

    for pin in REAR_PINS:
        GPIO.output(pin, GPIO.LOW)
