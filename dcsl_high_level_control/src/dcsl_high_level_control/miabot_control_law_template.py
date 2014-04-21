#!/usr/bin/env python

## @file miabot_control_law_template.py
#
#
#

## @author Will Scott

import numpy as np
import math as m
class MiabotControlLaw(object):
    def __init__(self, n_robots):
        ''' create any variables needed for persistant controller state'''
        self.n_robots = n_robots
        self.name = "miabot_control_law_template"

    def get_initial_conds(self):
    	''' return desired initial conditions for various numbers of robots 
    	for reference, here are the default initial miabot poses:'''
        defaults = np.array([
            [2.56, -1.46, m.pi],
            [2.56, -1.20, m.pi],
            [2.56, -0.98, m.pi],
            [2.56, -0.74, m.pi],
            [2.56, -0.50, m.pi],
            [2.56, -0.26, m.pi],
            [2.56, -0.03, m.pi]])
        return np.array(defaults[0:self.n_robots,:])


    def control_type(self):
    	''' return 1 for velocity control, 2 for waypoint control'''
        return 1

    def velocity_controller(self, t, x):
    	''' control law to calculate desired velocity from state estimate
        inputs: t is the time of most recent state estimate x,
                x is an n_robots-by-3 numpy array, with [x, y, theta] in each row
        returns: u, an n_robots-by-2 numpy array, with [v_x, omega] in each row'''
        return np.zeros([n_robots,2])

    def waypoint_controller(self, t, x):
    	''' control law to calculate desired waypoints from state estimate
        inputs: t is the time of most recent state estimate x,
                x is an n_robots-by-3 numpy array, with [x, y, theta] in each row
        returns: wp, an n_robots-by-3 numpy array of waypoints in same format as x'''
        return np.zeros([n_robots,3])

    def parameter_info(self):
        ''' return a dictionary with info about the parameters used in the control law'''
        return {"n_robots":self.n_robots}