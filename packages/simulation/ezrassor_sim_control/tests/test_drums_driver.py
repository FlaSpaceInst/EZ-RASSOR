"""A Python Unit Testing File for the Drums Driver

Written by Ronald Marrero.
Part of the EZ-RASSOR suite of software.
""" 
import unittest
from ezrassor_sim_control.drums_driver import get_movements

""" Global Mask and Test Cases Representing Drum Commands
1: Back Drum Down
2: Back Drum Up
4: Front Drum Down
8: Front Drum Up
"""
MASK = 0b000000001111
TEST_CASES = [(1, (False, False, False, True)),
             (2, (False, False, True, False)), 
             (4, (False, True, False, False)),
             (8, (True, False, False, False))]

class DrumsDriverTest(unittest.TestCase):

    """ Given a particular set of data, validate the output tuples against
    the expected outputs
    """
    def test_get_movements(self): 
        print "Validating Drum Movement Interpreter"
        for (data, expected_output) in TEST_CASES: 
            module_output = get_movements(data, MASK)
            self.assertEqual(module_output, expected_output) 
