#!/usr/bin/env python

## @file
# 
#
#

## @author Brendan Andrade

import math as m
import numpy as np
from scipy.integrate import odeint, quadrature

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
        self._x_hat_plus = self._ra_to_cm(init_x)
        self._P_plus = np.asmatrix(init_P)
        self._u = []

        self._f = f
        self._h = h
        self._F = F
        self._G = G
        self._H = H
        self._L = L
        self._Q = np.asmatrix(Q)
        self._R = np.asmatrix(R)
      
    ##
    #
    #
    def estimate(self, t, z):
        z = self._ra_to_cm(z)
        x_hat_minus = self._ra_to_cm(self.look_forward(t))
        P_minus = self._propagate_covariance_estimate(t)
        K = self._calculate_filter_gain(t, x_hat_minus, P_minus)
        x_hat_plus = self._update_state_estimate(t, x_hat_minus, z, K)
        P_plus = self._update_covariance(t, x_hat_minus, P_minus, K)
        self._x_hat_plus = x_hat_plus
        self._P_plus = P_plus
        self._t = t
        self._u = [self._u[-1]]
        return x_hat_plus # Change to return 1D array
        
    
    ##
    #
    #
    def look_forward(self, t, input_history = None):
        x_hat_plus = self._cm_to_ra(self._x_hat_plus)
        inputs = self._u
        if input_history is not None:
            inputs.append(input_history)
        if t is self._t:
            x_next = [x_hat_plus]
        else:
            t_array = np.array([self._t, t])
            x_next = x_hat_plus + odeint(self._state_propagation_integral_function, x_hat_plus, t_array, args=(inputs,)) # Check output for x_hat_plus != 0
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
        # t_array = np.array([self._t, t])
        if t is not self._t:
            P_k_minus = self._P_plus + self._trap_mat(self._covar_propagation_integral_function, self._t, t, args=(self._u,))
        else:
            P_k_minus = self._P_plus
        return P_k_minus
    
    ##
    #
    #
    def _calculate_filter_gain(self, t, x_hat_minus, P_k_minus):
        H = np.asmatrix(self._H(self._cm_to_ra(x_hat_minus), t))
        R = self._R
        K = P_k_minus*H.T*np.linalg.inv(H*P_k_minus*H.T + R)
        return K

    ##
    #
    #
    def _update_state_estimate(self, t, x_hat_minus, z, K):
        z_hat = self._ra_to_cm(self._h(self._cm_to_ra(x_hat_minus), t))
        x_hat_plus = x_hat_minus + K*(z - z_hat)
        return x_hat_plus
    
    ##
    #
    #
    def _update_covariance(self, t, x_hat_minus, P_k_minus, K):
        H = np.asmatrix(self._H(self._cm_to_ra(x_hat_minus), t))
        P_k_plus = (np.eye(K.shape[0]) - K*H)*P_k_minus
        return P_k_plus

    ##
    #
    #
    def _state_propagation_integral_function(self, x, t, input_history):
        # Determine the value of u at t
        u = self._u_at_t(t, input_history)

        #Determine x_dot
        x_dot = self._f(x, u, t)
        return x_dot

    ##
    #
    #
    def _covar_propagation_integral_function(self, t, input_history):
        u = self._u_at_t(t, input_history) # row_array
        x = self.look_forward(t) # row array
        F = np.asmatrix(self._F(x, u, t))
        L = np.asmatrix(self._L(x, u, t))
        Q = self._Q
        P_plus = self._P_plus
        integrand = F*P_plus + P_plus*F.T + L*Q*L.T # .T is transpose method for numpy matrix objects
        return integrand
        
    ##
    #
    #
    def _u_at_t(self, t, input_history):
        if len(input_history) is 1:
            u = input_history[0][1]
        elif t < input_history[0][0]:
            u = input_history[0][1]
        elif t >= input_history[-1][0]:
            u = input_history[-1][1]
        else:
            for i in range(len(input_history)-1):
                if t >= input_history[i][0] and t < input_history[i+1][0]:
                    u = input_history[i][1]
        return u
        
    ## 
    #
    #
    def _cm_to_ra(self, x):
        return np.squeeze(np.asarray(x.T))

    ##
    #
    #
    def _ra_to_cm(self, x):
        return np.asmatrix(x).T

    def _trap_mat(self, func, a, b, args=()):
        steps = 10
        t_array = np.linspace(a, b, steps)
        y = 0
        for i in xrange(0, steps-1):
            y = y + (t_array[i+1]-t_array[i])*(func(t_array[i], *args) + func(t_array[i+1], *args))/2.0 
        return y
