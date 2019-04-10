"""A Python Unit Testing File for the Wheels Driver

Written by Ronald Marrero.
Part of the EZ-RASSOR suite of software.
"""

import unittest
from ezrassor_sim_control.wheels_driver import get_movements

""" Global Mask and Test Cases Representing Wheel Commands
1280: Move Backward
1536: Move Left
2304: Move Right
2560: Move Forward
"""
MASK = 0b111100000000
TEST_CASES = [(1280, (False, True, False, True)), 
             (1536, (False, True, True, False)), 
             (2304, (True, False, False, True)),
             (2560, (True, False, True, False))]

class WheelsDriverTest(unittest.TestCase):

    """ Given a particular set of data, validate the output tuples against
    the expected outputs
    """
    def test_get_movements(self): 
        print "Validating Wheel Movement Interpreter"
        for (data, expected_output) in TEST_CASES: 
            module_output = get_movements(data, MASK)
            self.assertEqual(module_output, expected_output) 

