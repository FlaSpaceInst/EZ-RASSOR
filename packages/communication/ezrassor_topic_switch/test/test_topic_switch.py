#!/usr/bin/env python

import rostest
import unittest

class TestTest(unittest.TestCase):
    def test_one_is_one(self):
        self.assertEquals(1, 1, "1 doesnt equal 1")

rostest.rosrun("test_topic_switch", "test_test", TestTest)
