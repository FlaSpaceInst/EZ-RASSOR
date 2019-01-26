"""A collection of utilities that are shared among all moving parts nodes.

Written by Tiger Sachse and Harrison Black.
Part of the EZ-RASSOR suite of software.
"""
import threading

class StoppableThread(threading.Thread):
    """"""
    def __init__(self, *args, **kwargs):
        """"""
        super(StoppableThread, self).__init__(*args, **kwargs)
        self.__stop_flag = threading.Event()

    def stop(self):
        """"""
        self.__stop_flag.set()

    def is_stopped(self):
        """"""
        return self.__stop_flag.is_set()


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
