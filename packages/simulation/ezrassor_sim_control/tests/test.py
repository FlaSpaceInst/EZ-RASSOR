"""A Python Unit Testing Suite for the Simulation Controls

Written by Ronald Marrero.
Part of the EZ-RASSOR suite of software.
"""
import unittest
from test_arms_driver import ArmsDriverTest
from test_wheels_driver import WheelsDriverTest
from test_drums_driver import DrumsDriverTest

""" Add and execute all the tests in this module """
def main():
    tests = unittest.TestSuite()

    tests.addTest(unittest.makeSuite(ArmsDriverTest))
    tests.addTest(unittest.makeSuite(WheelsDriverTest))
    tests.addTest(unittest.makeSuite(DrumsDriverTest))

    return tests

if __name__ == "__main__":
    main()
