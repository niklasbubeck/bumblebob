#!/usr/bin/env python
import sys
import unittest

import roslib

from bumblebob_lateral_lq_controller.lateral_lqr_controller_node import *

PKG = 'bumblebob_lateral_lq_controller'


# A sample python unit test
class TestBareBones(unittest.TestCase):

    def test_one_equals_one(self):
        self.assertEquals(1, 1, "1!=1")

    def test_calculateCF(self):
        lqr = LateralLqrControllerNode()
        cf = lqr.defineMatrixA(1, 2, 3)
        self.assertEquals(1, 1, "1!=1")


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_bare_bones', TestBareBones)
