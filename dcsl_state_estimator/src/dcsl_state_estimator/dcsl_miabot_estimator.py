#!/usr/bin/env python

## @file
#

## @author Brendan Andrade

import rospy
from geometry_msgs.msg import PoseArray
from dcsl_messages.msg import TwistArray, StateArray, State
from dcsl_state_estimator.ekf_API import ekf
from dcsl_miabot_main.miabot import Miabot

import numpy as np
import math as m
import sys
##
#
#
class MiabotEstimator:
    
    ##
    #
    #
    def __init__(self, n_robots):
        self.n_robots = n_robots
        
        self.measurement_sub = rospy.Subscriber("planar_measurements", PoseArray, 
                                                self.measurement_callback, queue_size = 1)
        self.input_sub = rospy.Subscriber("cmd_vel_array", TwistArray, self.input_callback, queue_size = 1)
        self.pub = rospy.Publisher("state_estimate", StateArray)

        self.init_P = np.ones((8,8))*0.1
        self.init_u = np.array([0., 0., 0.])
        self.Q = np.identity(8)*0.001
        self.R = np.identity(5)*0.001
        self.R[3][3] = 0.01
        self.R[4][4] = 0.01

        self.current_u = np.zeros((self.n_robots,3))

        self.ekfs = [None]*7
        self.miabot = Miabot()
    ##
    #
    #
    def measurement_callback(self, data):
        t = float(data.header.stamp.secs) + float(data.header.stamp.nsecs)*pow(10.,-9)
        state_array = StateArray()
        for i, pose in enumerate(data.poses):
            # If robot was detected:
            if (pose.orientation.w == 1.0):
                z = self._pose_to_z_array(pose)
                # Initialize ekf for robot if necessary
                if self.ekfs[i] is None:
                    state_estimate = np.array([z[0], z[1], z[2], 0., 0., m.sin(z[3]), m.cos(z[3]), 0.]) #Assume initial velocities are zero
                    self.ekfs[i] = ekf(t, state_estimate, self.init_P, 
                                       self.init_u, self.miabot.f, self.miabot.h, 
                                       self.miabot.F, self.miabot.G, self.miabot.H, 
                                       self.miabot.L, self.Q, self.R)
                else:
                    state_estimate = self.ekfs[i].estimate(t, z)
        '''
            #On no measurement
            else:
                #If estimator initialized, just propagate state
                if self.ekfs[i] is not None:
                    state_estimate = self.ekfs[i].look_forward(t)
                # Otherwise  estimate is zeros.
                else:
                    state_estimate = np.zeros(8)
                    state_estimate[6] = 1.0
                    
            state = self._x_array_to_state(state_estimate, self.current_u)

            #Signify no estimate (due to tracking not started)
            if pose.orientation.w is not 1 and self.ekfs[i] is None:
                state.pose.orientation.w = 0
            else:
                state.pose.orientation.w = 1

            state_array.states.append(state)
        #Pass time to state message
        state_array.header.stamp = data.header.stamp
        self.pub.publish(state_array)
        '''
    ##
    #
    #
    def input_callback(self, data):
        t = float(data.header.stamp.secs) + float(data.header.stamp.nsecs)*pow(10.,-9)
        for i, twist in enumerate(data.twists):
            u = self._twist_to_u_array(twist)
            self.current_u[i] = u
            if self.ekfs[i] is not None: #Check if ekf is initialized for robot
                self.ekfs[i].update_u(t, u)

    ##
    #
    #
    def publish_estimate(self):
        no_estimates = True
        state_array = StateArray()
        now = rospy.get_rostime()
        t = rospy.get_time()
        for i in xrange(0, self.n_robots):
            # If not initialized
            if self.ekfs[i] is None:
                state_estimate = np.zeros(8)
                state = self._x_array_to_state(state_estimate, self.current_u[i])
                state.pose.orientation.w = 0
            else:
                no_estimates = False
                state_estimate = self.ekfs[i].look_forward(t)
                state = self._x_array_to_state(state_estimate, self.current_u[i])
                state.pose.orientation.w = 1
            state_array.states.append(state)
        
        state_array.header.stamp = now
        if not no_estimates:
            self.pub.publish(state_array)

    ##
    #
    #
    def _pose_to_z_array(self, pose):
        z = np.zeros(5)
        z[0] = pose.position.x
        z[1] = pose.position.y
        z[2] = pose.position.z
        z[3] = m.sin(pose.orientation.z)
        z[4] = m.cos(pose.orientation.z)
        return z

    ##
    #
    #
    def _twist_to_u_array(self, twist):
        u = np.zeros(3)
        u[0] = twist.linear.x
        u[1] = twist.angular.z
        u[2] = twist.linear.z
        return u

    ##
    #
    #
    def _x_array_to_state(self, x, u):
        state = State()
        state.pose.position.x = x[0]
        state.pose.position.y = x[1]
        state.pose.position.z = x[2]
        state.pose.orientation.z = m.atan2(x[5],x[6])
        state.twist.linear.x = u[0]
        state.twist.linear.z = u[2]
        state.twist.angular.z = u[1]
        return state
    
def main():
    rospy.init_node('dcsl_miabot_estimator')

    n_robots = int(sys.argv[1])

    estimator = MiabotEstimator(n_robots)

    r = rospy.Rate(20)
 
    while not rospy.is_shutdown():
        estimator.publish_estimate()
        r.sleep()

if __name__ == '__main__':
    main()
