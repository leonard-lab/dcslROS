#!/usr/bin/env python
PKG = 'dcsl_vision_tracker'
import roslib; roslib.load_manifest(PKG)

import sys
import unittest

class TestVisionTracker(unittest.TestCase):
    # Description
    def test_foo(self):
        self.assertEquals(1,1, "Oh no!")

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'test_dcsl_vision_tracker', TestVisionTracker)
