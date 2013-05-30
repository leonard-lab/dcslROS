#!/usr/bin/env python

## @file
# Estimator node for Beluga UUV.
#
#

## @author Brendan Andrade

from ukf_API import ukf

import numpy as np
import math as m

import rospy
from geometry_msgs.msg import PoseArray, Pose
from geometry_msgs.msg import Twist
from dcsl_messages.msg import TwistArray

###############################################

##
class BelugaEstimator(object):
    
    ##
    def __init__(self):
        self.eta_t = 0.1
        self.eta_z_up = 0.1
        self.eta_z_down = 0.1
        self.Kt = 0.2
        self.Kd1 = 50.
        self.Kd3 = 70.
        self.K_omega = 12.
        self.J = 2.5
        self.Kg = 0.9
        self.z_neutral = 1.26
        self.m1 = 6
        self.m3 = 4
        self.r = 0.35
        self.Ktau = 0.05

        Q = np.eye(7, dtype=float)
        R = np.eye(4, dtype=float)
        x_init = np.zeros(7)

        self.ukf = ukf(self.f, self.g, Q, R, x_init, 7, 3, 4, 0.1)
    
    ## 
    def f(self, state, t, command):
        x = state[0]
        y = state[1]
        z = state[2]
        v1 = state[3]
        v3 = state[4]
        theta = state[5]
        theta_dot = state[6]
        
        u_t = command[0]
        u_phi = command[1]
        u_z = command[2]

        x_dot = v1*m.cos(theta)
        y_dot = v1*m.sin(theta)
        z_dot = v3
        
        F1 = (1-self.eta_t)*self.Kt*u_t*m.cos(u_phi) - self.Kd1*v1*abs(v1)
        if v3 < 0:
            eta_z = self.eta_z_down
        else:
            eta_z = self.eta_z_up
        F3 = (1-eta_z)*self.Kt*u_z - self.Kd3*v3*abs(v3) + self.Kg*(self.z_neutral - z)
        
        v1_dot = F1/self.m1
        v3_dot = F3/self.m3

        gamma = (1-self.eta_t)*self.Kt*m.sin(u_phi)*self.r - self.K_omega*theta_dot*abs(theta_dot) - self.Ktau*u_z

        theta_dotdot = gamma/self.J

        return np.array([x_dot, y_dot, z_dot, v1_dot, v3_dot, theta_dot, theta_dotdot])

    def g(self, state):
        return np.array([state[0], state[1], state[2], state[5]])
    
