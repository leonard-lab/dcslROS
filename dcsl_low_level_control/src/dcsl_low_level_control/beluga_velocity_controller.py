#!/usr/bin/env python

## @file
#
#
#

## @author Brendan Andrade

import numpy as np
import math as m
from scipy.interpolate import griddata

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
        self.n_states = 7
        self.n_inputs = 4
        self.K_array = K_array
        self.x3_array = x3_array
        self.x4_array = x4_array 
        self.x5_array = x5_array
        self.x7_array = x7_array
        self.Ki = np.asmatrix(Ki)

    ##
    #
    #
    def control_law(self, x, xi, vel_goal):
        x_goal_reduced = np.asmatrix(vel_goal).transpose()
        x_reduced = np.matrix([[x[2]], [x[3]], [x[4]], [x[6]]])
        xi_col = np.asmatrix(xi).transpose()
        u_nominal = np.asmatrix(self._calc_u_nominal(vel_goal)).transpose()
        K_star = np.asmatrix(self._interpolate_K(vel_goal))
        u_out = -K_star*(x_reduced - x_goal_reduced) + u_nominal - self.Ki*xi_col
        return u_out

    ##
    #
    #
    def _interpolate_K(self, x_reduced):
        K_out = np.zeros(self.inputs, self.n_states)
        for i in xrange(0, self.n_states):
            for j in xrange(0, self.n_inputs):
                K_out(i,j) = griddata((self.x3_array, self.x4_array, self.x5_array, self.x7_array), self.K_array[:,:,:,:,i,j], x_reduced, 'linear')
        return K_out

    ##
    #
    #
    def _calc_u_nominial(self, x_goal):
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
            u2 = m.atan(-(self.beluga.KOmega*x7*abs(x7) + self.beluga.Kdz*u3)/(self.beluga.Kd1*self.beluga.r*x4*abs(x4)))

        if abs(u2) < 0.000001:
            u1 = (self.beluga.Kd1*x4*abs(x4))/((1.0-self.beluga.eta1)*self.beluga.Kt)
        else:
            u1 = -(self.beluga.KOmega*x7*abs(x7) + self.beluga.Kdz*u3)/((1.0-self.beluga.eta1)*self.beluga.Kt*self.beluga.r*m.sin(u2))

        return np.array([u1, u2, u3])
        
