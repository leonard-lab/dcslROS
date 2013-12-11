#!/usr/bin/env python

## @file miabot_logger.py
#
#
#

## @author Will Scott

import numpy as np
import math as m
import scipy.io
import time

class MiabotLogger(object):
    def __init__(self, n_robots, control_type):
        self.n = n_robots

        # control type is 1 for velocity (2 inputs/bot), 2 for waypoint (3 inputs/bot)
        self.u_dim = control_type + 1

        self.t_list = []
        self.x_list = []
        self.u_list = []
        
        self._arraysize = 10000 # how many measurements to keep in each array

        self.i_list = 0 # current position of arrays in list that are being filled
        self.i = 0  # where to place next measurement in the arrays
        
        self._add_new_arrays() # place the first array into each list

    def _add_new_arrays(self):
        self.t_list.append(np.empty([self._arraysize]))
        self.x_list.append(np.empty([self._arraysize, self.n, 3]))
        self.u_list.append(np.empty([self._arraysize, self.n, self.u_dim]))

    def add_to_log(self, t, x, u):
        if self.i == self._arraysize:
            self._add_new_arrays
            self.i_list += 1
            self.i = 0

        self.t_list[self.i_list][self.i] = t
        self.x_list[self.i_list][self.i,:,:] = x
        self.u_list[self.i_list][self.i,:,:] = u
        self.i += 1
        
    def export_to_mat(self, info_dict={}):
        # exports t, x, and u to a matlab-formatted .mat file, with date as filename
        # optional argument info_dict contains extra variables with info about control law

        timestr = time.strftime("%Y%m%d-%H%M%S") # string of 'year month day - hour min sec'
        filename = timestr + '.mat'

        # trim the data in last array and put all arrays together
        self.t_list[-1] = self.t_list[-1][0:self.i]
        self.x_list[-1] = self.x_list[-1][0:self.i,:,:]
        self.u_list[-1] = self.u_list[-1][0:self.i,:,:]
        
        big_t_list = np.concatenate(self.t_list)
        big_x_list = np.concatenate(self.x_list)
        big_u_list = np.concatenate(self.u_list)

        # remove time offset so that t[0] = 0
        big_t_list = big_t_list - big_t_list[0]

        variables_dict = {'t':big_t_list, 'x':big_x_list, 'u':big_u_list}
        # add extra variables to the dictionary if they were passed
        if info_dict:
            variables_dict.update(info_dict)

        scipy.io.savemat(filename,variables_dict)

        return filename
        