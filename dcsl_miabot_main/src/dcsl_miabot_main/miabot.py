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
    self.max_motor_speed = 1.0 # Maximum motor speed set by miabot_driver in m/s
    self.d = 0.1 # Distance between the wheels in meters

    ##
    #
    #
    def __init__(self):
        pass
    
    ##
    #
    #
    def f(x, u, t):
        x_dot = np.zeros(7)
        x_dot[0] = u[0]*m.cos(x[5])
        x_dot[1] = u[0]*m.sin(x[5])
        x_dot[2] = u[2]
        x_dot[3] = 0.
        x_dot[4] = 0.
        x_dot[5] = u[1]
        x_dot[6] = 0.
        return x_dot
    
    ##
    #
    #
    def h(x, t):
        y = np.zeros(4)
        y[0] = x[0]
        y[1] = x[1]
        y[2] = x[2]
        y[3] = x[5]
        return y

    ##
    #
    #
    def F(x, u, t):
        F = np.zeros((7, 7))
        F[0][5] = u[0]*-1.0*m.sin(x[5])
        F[1][5] = u[0]*m.cos(x[5])
        return F
    
    ##
    #
    #
    def G(x, u, t):
        G = np.zeros((7,3))
        G[0][0] = m.cos(x[5])
        G[1][0] = m.sin(x[5])
        G[5][1] = 1.
        return G

    ##
    #
    #
    def H(x, t):
        H = np.zeros((4,7))
        H[0][0] = 1.
        H[1][1] = 1.
        H[2][2] = 1.
        H[3][5] = 1.
        return H

    ##
    #
    #
    def L(x, u, t):
        L = np.identity(7)
        return L
