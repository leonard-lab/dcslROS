#!/usr/bin/env python

## @file
#
#
#

## @author Brendan Andrade

import numpy as np
import math as m
from scipy.ndimage.interpolation import map_coordinates

from dcsl_beluga_main.beluga import Beluga

##
#
#
class BelugaVelocityController(object):

    ##
    #
    #
    def __init__(self, K_array, x3_array, x4_array, x5_array, x7_array, Ki):
        self.beluga = Beluga()
        self.n_states = 4
        self.n_inputs = 3
        self.K_array = K_array
        self.x3_array = x3_array #Increasing with consistant step required
        self.x4_array = x4_array 
        self.x5_array = x5_array
        self.x7_array = x7_array
        self.Ki = np.asmatrix(Ki)

    ##
    #
    #
    def control_law(self, x, xi, vel_goal):
        vel_goal = np.insert(vel_goal, 0, x[2]) # Pad with z goal = z current
        x_goal_reduced = np.asmatrix(vel_goal).transpose()
        x_reduced = np.matrix([[x[2]], [x[3]], [x[4]], [x[7]]])
        xi_col = np.asmatrix(xi).transpose()
        u_nominal = np.asmatrix(self._calc_u_nominal(vel_goal)).transpose()
        K_star = np.asmatrix(self._interpolate_K(vel_goal))
        u_out = -K_star*(x_reduced - x_goal_reduced) + u_nominal - self.Ki*xi_col
        return u_out

    ##
    #
    #
    def _interpolate_K(self, x_reduced):
        K_out = np.zeros((self.n_inputs, self.n_states))
        x_reduced_norm = self._normalize_x_reduced(x_reduced)
        for i in xrange(0, self.n_inputs):
            for j in xrange(0, self.n_states):
                [K_out[i,j],_] = map_coordinates(self.K_array[:,:,:,:,i,j],np.array([x_reduced_norm, np.ones(4)*0.1]).T, mode='nearest')
        return K_out

    ##
    #
    #
    def _normalize_x_reduced(self, x_reduced):
        norm_x = np.zeros(4)
        grid = np.array([self.x3_array, self.x4_array, self.x5_array, self.x7_array])
        for i, entry in enumerate(x_reduced):
            norm_x[i] = (entry-grid[i,0])/(grid[i,-1]-grid[i,0])*(len(grid[i])-1)
        return norm_x

    ##
    #
    #
    def _calc_u_nominal(self, x_goal):
        x3 = x_goal[0]
        x4 = x_goal[1]
        x5 = x_goal[2]
        x7 = x_goal[3]
        
        if x5 >= 0:
            eta3 = self.beluga.eta3up
        else:
            eta3 = self.beluga.eta3down

        u3 = (self.beluga.Kd3*x5*abs(x5) - self.beluga.Kg*(self.beluga.z_neutral - x3))/((1.0-eta3)*self.beluga.Kt)
        if abs(x4) < 0.000001:
            u2 = m.copysign(m.pi*0.5, -x7)
        else:
            u2 = m.atan(-(self.beluga.K_omega*x7*abs(x7) + self.beluga.Ktau*u3)/(self.beluga.Kd1*self.beluga.r*x4*abs(x4)))

        if abs(u2) < 0.000001:
            u1 = (self.beluga.Kd1*x4*abs(x4))/((1.0-self.beluga.eta1)*self.beluga.Kt)
        else:
            u1 = -(self.beluga.K_omega*x7*abs(x7) + self.beluga.Ktau*u3)/((1.0-self.beluga.eta1)*self.beluga.Kt*self.beluga.r*m.sin(u2))

        return np.array([u1, u2, u3])
        
