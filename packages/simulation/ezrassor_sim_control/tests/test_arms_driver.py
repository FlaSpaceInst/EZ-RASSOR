"""A Python Unit Testing File for the Arms Driver

Written by Ronald Marrero.
Part of the EZ-RASSOR suite of software.
"""

import unittest
from ezrassor_sim_control.arms_driver import get_movements, handle_arm_movements

""" Global Mask and Test Cases Representing Arm Commands
16: Back Arm Down
32: Back Arm Up
64: Front Arm Down
128: Front Arm Up
"""
MASK = 0b000011110000 
TEST_CASES = [(16, (False, False, False, True)), 
             (32, (False, False, True, False)), 
             (64, (False, True, False, False)),
             (128, (True, False, False, False))]

class ArmsDriverTest(unittest.TestCase):
    
    """ Given a particular set of data, validate the output tuples against
    the expected outputs
    """
    def test_get_movements(self):
        print "Validating Arm Movement Interpreter"
        for (data, expected_output) in TEST_CASES: 
            module_output = get_movements(data, MASK)
            self.assertEqual(module_output, expected_output) 
