#!/usr/bin/env python

## @file miabot_control_law_template.py
#
#
#

## @author Will Scott

import numpy as np
import math as m
class MiabotControlLaw(object):
    def __init__(self):
        ''' create any variables needed for persistant controller state'''
        self.variable = 0

    def get_initial_conds(self, n_robots):
    	''' return desired initial conditions for various numbers of robots 
    	for reference, here are the default initial miabot poses:
        [2.56, -1.46, 0.]
        [2.56, -1.20, 0.]
        [2.56, -0.98, 0.]
        [2.56, -0.74, 0.]
        [2.56, -0.50, 0.]
        [2.56, -0.26, 0.]
        [2.56, -0.03, 0.]'''
        return np.zeros([n_robots,3])


    def control_type(self):
    	# return 1 for velocity control, 2 for waypoint control
        return 1

    def velocity_controller(self, t, x):
    	''' control law to calculate desired velocity from state estimate
        inputs: t is the time of most recent state estimate x,
                x is an n_robots-by-3 numpy array, with [x, y, theta] in each row
        returns: u, an n_robots-by-2 numpy array, with [v_x, omega] in each row'''
        return np.zeros([n_robots,3])

    def waypoint_controller(self, t, x):
    	''' control law to calculate desired waypoints from state estimate
        inputs: t is the time of most recent state estimate x,
                x is an n_robots-by-3 numpy array, with [x, y, theta] in each row
        returns: wp, an n_robots-by-3 numpy array of waypoints in same format as x'''
        return np.zeros([n_robots,3])