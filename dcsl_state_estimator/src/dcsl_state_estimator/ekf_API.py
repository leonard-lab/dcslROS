#!/usr/bin/env python

## @file
# 
#
#

## @author Brendan Andrade

import math as m
import numpy as np
from scipy.integrate import odeint

########################################

##
#
#
class ekf(object):
    
    ##
    #
    #
    def __init__(self, init_t, init_x, init_P, f, h, F, G, H, L, Q, R):
        self._t = init_t
        self._x_hat_plus = init_x
        self._P_plus = init_P
        self._u = []

        self.f = f
        self.h = h
        self.F = F
        self.G = G
        self.H = H
        self.L = L
        self.Q = Q
        self.R = R
      
    ##
    #
    #
    def estimate(self, t, z):
        pass
        
    
    ##
    #
    #
    def propagate_state(self, t, input_history = None):
        inputs = self._u
        if input_history is not None:
            inputs.append(input_history)
        t_array = np.array([self._t, t])
        x_next = self._x_hat_plus + odeint(self._state_propagation_integral_function, self._x_hat_plus, t_array, args=(inputs,))
        return x_next[-1]

    ## 
    #
    #
    def update_u(self, t, u):
        self._u.append((t, u))

    ##
    #
    #
    def _propagate_covariance(self, x, u, t, dt, p):
        F_t = self.F(x, u, t);
        p_plus = p + dt*(F_t*p + p*np.transpose(F_t) + obj.Q) #Is this right?
        return p_plus
    
    ##
    #
    #
    def _calculate_filter_gain(self, p):
        k = np.inverse(p + self.R)
        return k

    ##
    #
    #
    def _update_state(self, x, y, k):
        x_plus = x + k*(y - x)
        return x_plus
    
    ##
    #
    #
    def _update_covariance(self, p, k):
        p_plus = (np.identity(k.shape[1]) - k)*p
        return p_plus

    def _state_propagation_integral_function(self, x, t, input_history):
        # Determine the value of u at t
        u = self._u_at_t(t, input_history)

        #Determine x_dot
        x_dot = self.f(x, u, t)
        return x_dot

    def _covar_propagation_integral_function(self, p, t, input_history):
        u = self._u_at_t(t, input_history)
        

    def _u_at_t(self, t, input_history):
        if len(input_history) is 1:
            u = input_history[0][1]
        elif t < input_history[0][0]:
            u = input_history[0][1]
        elif t > input_history[-1][0]:
            u = input_history[-1][1]
        else:
            for i in range(len(input_history)-1):
                if t >= input_history[i][0] and t < input_history[i+1][0]:
                    u = input_history[i][1]
        return u
        
