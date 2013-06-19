#!/usr/bin/env python

from nose.tools import *
from dcsl_state_estimator.ekf_API import ekf
import numpy as np

class TestEKF:
    
    @classmethod
    def setup_class(cls):
        pass
    
    @classmethod
    def teardown_class(cls):
        pass

    @raises(Exception)
    def test_size_init_P(self):
        e = ekf(0, np.zeros(2), np.zeros((3,3)), 
        assert_equals(1, 2)
