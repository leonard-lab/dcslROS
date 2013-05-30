#!/usr/bin/env python

## @file
# This API provides a wrapper around the pykalman library for state estimation.
#
# See pykalman.github.io for information about the pykalman library.

## @author Brendan Andrade

import math as m
import numpy as np
from scipy import integrate
from pykalman import AdditiveUnscentedKalmanFilter

#################################################

##
#
#
class ukf(object):
    
    ## Class initializer.
    #
    #
    def __init__(self, f, g, Q, R, initial_state, n_state, n_input, n_output, Ts, initial_input = None, integration_steps = 10):
        self._f = f
        self._g = g
        self._Q = Q
        self._R = R
        self._Ts = float(Ts)
        self._ode_steps = integration_steps

        if initial_input is not None:
            u = initial_input
        else:
            u = np.zeros(n_input)
        self._previous_u = u
        self._tfs = []
        
        self._measurements = [[initial_state[0], initial_state[1], initial_state[2], initial_state[5]]]

        self._aukf = AdditiveUnscentedKalmanFilter(transition_functions=self._tfs, observation_functions=self._observation_function, transition_covariance = self._Q, observation_covariance = self._R, initial_state_mean=initial_state, n_dim_state=n_state, n_dim_obs=n_output)
        

    def estimate(self):
        means, covar = self._aukf.filter(self._measurements)
        return means[-1:,]

    def update(self, y, u):
        self._measurements.append(y)
        self._tfs.append(lambda x: self._transition_function(x, self._previous_u))
        self._previous_u = u
        self._aukf.transition_functions = self._tfs

    def set_Q(self, Q):
        self._aukf.transition_covariance = Q
        self._Q = Q

    def set_R(self, R):
        self._aukf.observation_covariance = R
        self._R = R
    
    def _transition_function(self, state, u):
        step = float(self._Ts - 0.)/float(self._ode_steps)
        t = np.arange(0.,self._Ts+step,step)
        x_next = integrate.odeint(self._f, state, t, args=(u,))
        return x_next[self._ode_steps]

    def _observation_function(self, state):
        return self._g(state)

    
