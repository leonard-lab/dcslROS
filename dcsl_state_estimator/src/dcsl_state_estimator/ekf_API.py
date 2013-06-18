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
        x_hat_minus = self.look_forward(t)
        P_minus = self._propagate_covariance_estimate(t)
        K = self._calculate_filter_gain(t, x_hat_minus, P_k_minus)
        x_hat_plus = self._update_state_estimate(t, x_hat_minus, z, K)
        P_plus = self._update_covariance(t, x_hat_minus, P_minus, K)
        self._x_hat_plus = x_hat_plus
        self._P_plus = P_plus
        self._t = t
        self._u = self._u[-1]
        return x_hat_plus
        
    
    ##
    #
    #
    def look_forward(self, t, input_history = None):
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
    def _propagate_covariance_estimate(self, t):
        t_array = np.array([self._t, t])
        P_k_minus = self._P_plus + odeint(self._covar_propagation_integral_function, self._P_plus, t_array, args=(obj._u,))
        return P_k_minus[-1]
    
    ##
    #
    #
    def _calculate_filter_gain(self, t, x_hat_minus, P_k_minus):
        H = self.H(x_hat_minus, t)
        K = P_k_minus*np.transpose(H)*np.inverse(H*P_k_minus*np.transpose(H) + self.R)
        return K

    ##
    #
    #
    def _update_state_estimate(self, t, x_hat_minus, z, K):
        x_hat_plus = x_hat_minus + K*(z - self.h(x_hat_minus, t))
        return x_hat_plus
    
    ##
    #
    #
    def _update_covariance(self, t, x_hat_minus, P_k_minus, K):
        P_k_plus = (np.identity(K.shape[1]) - K*self.H(x_hat_minus, t))*P_k_minus
        return P_k_plus

    ##
    #
    #
    def _state_propagation_integral_function(self, x, t, input_history):
        # Determine the value of u at t
        u = self._u_at_t(t, input_history)

        #Determine x_dot
        x_dot = self.f(x, u, t)
        return x_dot

    ##
    #
    #
    def _covar_propagation_integral_function(self, p, t, input_history):
        u = self._u_at_t(t, input_history)
        x = self.lookforward(t)
        F = self.F(x, u, t)
        P_dot = F*self._P_plus + self._P_plus*np.transpose(F) + self.L*self.Q*np.transpose(self.L)
        return P_dot
        
    ##
    #
    #
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
        
