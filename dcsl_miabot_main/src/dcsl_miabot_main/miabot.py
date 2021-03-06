#!/usr/bin/env python

## @file
#
#

## @author Brendan Andrade

import math as m
import numpy as np

##
#
#
class Miabot(object):

    ##
    #
    #
    def __init__(self):
        self.max_motor_speed = 1.0 # Maximum motor speed set by miabot_driver in m/s
        self.d = 0.0667 # Distance between the wheels in meters
    
    ##
    #
    #
    def f(self, x, u, t):
        u = self._constrain_u(u)
        x_dot = np.zeros(8)
        x_dot[0] = u[0]*x[6]
        x_dot[1] = u[0]*x[5]
        x_dot[2] = u[2]
        x_dot[3] = 0.
        x_dot[4] = 0.
        x_dot[5] = u[1]*x[6]
        x_dot[6] = -u[1]*x[5]
        x_dot[7] = 0.
        return x_dot
    
    ##
    #
    #
    def h(self, x, t):
        y = np.zeros(5)
        y[0] = x[0]
        y[1] = x[1]
        y[2] = x[2]
        y[3] = x[5]
        y[4] = x[6]
        return y

    ##
    #
    #
    def F(self, x, u, t):
        u = self._constrain_u(u)
        F = np.zeros((8, 8))
        F[0][6] = u[0]
        F[1][5] = u[0]
        F[5][6] = u[1]
        F[6][5] = -u[1]
        return F
    
    ##
    #
    #
    def G(self, x, u, t):
        G = np.zeros((8,3))
        G[0][0] = x[6]
        G[1][0] = x[5]
        G[5][1] = x[6]
        G[6][1] = -x[5]
        return G

    ##
    #
    #
    def H(self, x, t):
        H = np.zeros((5,8))
        H[0][0] = 1.
        H[1][1] = 1.
        H[2][2] = 1.
        H[3][5] = 1.
        H[4][6] = 1.
        return H

    ##
    #
    #
    def L(self, x, u, t):
        L = np.identity(8)
        return L

    def _constrain_u(self, u):
        diffConversionFactor = self.d # meters
        max_motor_speed = self.max_motor_speed # m/s
        v_right = u[0] + u[1]*diffConversionFactor/2.
        v_left = u[0] - u[1]*diffConversionFactor/2.
        
        if v_right > max_motor_speed:
            v_right = max_motor_speed
        elif v_right < -max_motor_speed:
            v_right = -max_motor_speed

        if v_left > max_motor_speed:
            v_left = max_motor_speed
        elif v_left < -max_motor_speed:
            v_left = -max_motor_speed

        u[0] = (v_left + v_right)/2.
        u[1] = (v_right - v_left)/diffConversionFactor
        
        return u
