#!/usr/bin/env python

## @file
#
#
#

## @author Brendan Andrade

import math as m
import numpy as np

class Beluga(object):
    
    ##
    def __init__(self):
        self.eta1 = 0.73
        self.eta3up = 0.92
        self.eta3down = 0.94
        self.Kt = 0.17
        self.Kd1 = 66.
        self.Kd3 = 60.
        self.K_omega = 3.3
        self.J = 1.4
        self.Kg = 0.6
        self.z_neutral = 1.5
        self.m1 = 30.
        self.m3 = 15.
        self.r = 0.35
        self.Ktau = 0.004
    
    ## 
    def f(self, x, u, t):
        u = self._constrain_u(u)
        x_dot = np.zeros(8)


        F1 = (1.0-self.eta1)*self.Kt*u[0]*m.cos(u[1]) - self.Kd1*x[3]*abs(x[3])
   
        if x[4] < 0:
            eta3 = self.eta3down
        else:
            eta3 = self.eta3up
        F3 = (1.0-eta3)*self.Kt*u[2] - self.Kd3*x[4]*abs(x[4]) + self.Kg*(self.z_neutral-x[2])
        
        Gamma = -1.*(1.0-self.eta1)*self.Kt*u[0]*self.r*m.sin(u[1]) - self.K_omega*x[7]*abs(x[7]) - self.Ktau*u[2]

        x_dot[0] = x[3]*x[6]
        x_dot[1] = x[3]*x[5]
        x_dot[2] = x[4]
        x_dot[3] = F1/self.m1
        x_dot[4] = F3/self.m3
        x_dot[5] = x[7]*x[6]
        x_dot[6] = -x[7]*x[5]
        x_dot[7] = Gamma/self.J
	return x_dot

    ##
    def h(self, state, t):
        return np.array([state[0], state[1], state[2], state[5], state[6]])

    ##
    def F(self, x, u, t):
        u = self._constrain_u(u)

        F = np.zeros((8,8))
        F[0][3] = x[6]
        F[0][6] = x[3]
        F[1][3] = x[5]
        F[1][5] = x[3]
        F[2][4] = 1.0
        F[3][3] = -self.Kd1*2.0*abs(x[3])/self.m1
        F[4][2] = -self.Kg/self.m3
        F[4][4] = -self.Kd3*2.0*abs(x[4])/self.m3
        F[5][6] = x[7]
        F[5][7] = x[6]
        F[6][5] = -x[7]
        F[6][7] = -x[5]
        F[7][7] = -self.K_omega*2.0*abs(x[7])
        return F
        
    ##
    def G(self, x, u, t):
        u = self._constrain_u(u)
        
        if x[4] < 0:
            eta3 = self.eta3down
        else:
            eta3 = self.eta3up

        G = np.zeros((8,3))
        G[3][0] = (1.0-self.eta1)*self.Kt*m.cos(u[1])
        G[3][1] = (1.0-self.eta1)*self.Kt*u[0]*-m.sin(u[1])
        G[4][2] = (1.0-eta3)*self.Kt
        G[7][0] = -1.0*(1.0-self.eta1)*self.Kt*self.r*m.sin(u[1])
        G[7][1] = -1.0*(1.0-self.eta1)*self.Kt*u[0]*self.r*m.cos(u[1])
        G[7][2] = -self.Ktau
        return G

    ##
    def H(self, x, t):
        H = np.zeros((5,8))
        H[0][0] = 1.
        H[1][1] = 1.
        H[2][2] = 1.
        H[3][5] = 1.
        H[4][6] = 1.
        return H
        
    ##
    def L(self, x, u, t):
        L = np.identity(8)
        return L

    ##
    def _constrain_u(self, u):
        if abs(u[1]) > m.pi/2.0:
            u[1] = m.copysign(m.pi/2.0, u[1])

        if abs(u[0]) > 255:
            u[0] = m.copysign(255, u[0])
            
        if abs(u[2]) > 255:
            u[2] = m.copysign(255, u[2])
        return u
