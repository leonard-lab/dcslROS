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
        self.f = f
        self.g = g
        self.Q = Q
        self.R = R
        self.Ts = float(Ts)
        self.ode_steps = integration_steps

        if initial_input is not None:
            self.u = initial_input
        else:
            self.u = np.zeros(n_input)

        self.aukf = AdditiveUnscentedKalmanFilter(transition_function=self.transition_function, observation_function=self.observation_function, initial_state_mean=initial_state, n_dim_state=n_state, n_dim_obs=n_outout)
        



    def new_measurement(self, y):
        pass

    def new_input(self, u):
        self.u = u
    
    def transition_function(self, state):
        step = float(self.Ts - 0.)/float(self.ode_steps)
        t = np.arange(0.,self.Ts+step,step)
        x_next = integrate.odeint(self.f, state, t, args=(self.u,))
        return np.array(x_next)

    def observation_function(self, state):
        return self.g(state)

    
