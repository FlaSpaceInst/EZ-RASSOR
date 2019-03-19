"""A collection of utilities that are shared among all moving parts nodes.

Written by Tiger Sachse.
"""
import RPi.GPIO as GPIO


def get_toggles(bitstring, mask):
    """Retrieve individual toggles from the bitstring, using a given mask."""
    toggles = []

    # Extract toggles from the bitstring.
    while mask > 0:
        if mask & 0b1 == 1:
            toggles.append(bitstring & 0b1 == 1)
        bitstring >>= 1
        mask >>= 1

    # Return the toggles in reversed order (so the least significant toggle is
    # last in the list).
    return tuple(reversed(toggles))


def turn_off_pins(*pin_iterables):
    """Turn off all pins. Expects 1 or more iterables of pin integers."""
    for pin_iterable in pin_iterables:
        for pin in pin_iterable:
            GPIO.output(pin, GPIO.LOW)


def enqueue_toggles(instruction, additional_arguments):
    """Decode some toggles from the instruction and enqueue them.
    
    Arguments after instruction are passed as a single tuple because of how ROS
    callbacks work.
    """
    toggle_queue, mask, message_format, messages, default_message = additional_arguments

    toggles = get_toggles(instruction.data, mask)
    toggle_queue.put(toggles, False)

    print_status(message_format, toggles, messages, default_message)


def print_status(message_format, toggles, messages, default_message):
    """Print status information based on the provided toggles."""
    for toggle, message in zip(toggles, messages):
        if toggle:
            print message_format % message
    else:
        print message_format % default_message
