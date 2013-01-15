#!/usr/bin/env python
PKG = 'dcsl_miabot_driver'
import roslib; roslib.load_manifest(PKG)
from serial import Serial

import sys
import unittest

class TestMiabotDriver(unittest.TestCase):
    ## Test proper serial output given a cmd_vel message
    def test_serial_output(self):
        self.assertEquals()

    ## Test proper shutdown of node
    def test_shutdown(self):
        self.assert

    

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'miabot_driver_test',TestMiabotDriver)
