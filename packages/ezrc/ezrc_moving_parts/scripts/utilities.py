"""A collection of utilities that are shared among all moving parts nodes.

Written by Tiger Sachse and Harrison Black.
Part of the EZ-RASSOR suite of software.
"""
import RPi.GPIO as GPIO


def get_nibble(bitstring, mask):
    """Retrieve a nibble of data from the bitstring, using a given mask."""

    # Use a mask to remove unnecessary bits.
    nibble = bitstring & mask

    # Shift the result so the nibble is in the least significant position.
    # The shift amount is determined by the mask.
    while mask % 2 == 0:
        nibble >>= 1
        mask >>= 1

    # Return the nibble as 4 boolean values.
    return (
        nibble & 0b1000 != 0,
        nibble & 0b100 != 0,
        nibble & 0b10 != 0,
        nibble & 0b1 != 0
    )


def turn_off_pins(*pin_iterables):
    """Turn off all pins. Expects 1 or more iterables of pin integers."""
    for pin_iterable in pin_iterables:
        for pin in pin_iterable:
            GPIO.output(pin, GPIO.LOW)
