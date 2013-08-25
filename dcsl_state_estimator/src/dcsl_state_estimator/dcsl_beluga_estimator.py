#!/usr/bin/env python

## @file
# Estimator node for Beluga UUV.
#
#

## @author Brendan Andrade

import numpy as np
import math as m
import sys

import rospy
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Twist
from dcsl_messages.msg import TwistArray, StateArray, State, BelugaArray
from std_msgs.msg import Float32

from dcsl_beluga_main.beluga import Beluga
from dcsl_state_estimator.ekf_API import ekf

###############################################

##
class BelugaEstimator(object):
    
    ##
    def __init__(self):

        self.beluga = Beluga()

        self.init_P = np.ones((8,8))*0.1
        self.init_u = np.array([0., 0., 0.])
        self.Q = np.eye(8, dtype=float)
        self.R = np.eye(5, dtype=float)
        self.ekfs = [None]*4;

        self.depths = [0]*4
        depth_callback_list = [self.depth0_callback, self.depth1_callback, self.depth2_callback, self.depth3_callback]
        self.depth_sub_array = []
        for i,depth_callback in enumerate(depth_callback_list):
            name = "robot" + str(i) +"/depth_measurement"
            subscriber = rospy.Subscriber(name, Float32, depth_callback)
            self.depth_sub_array.append(subscriber)

        self.input_sub = rospy.Subscriber("/cmd_inputs", BelugaArray, self.input_callback)
        self.planar_sub = rospy.Subscriber("/planar_measurements", PoseArray, self.planar_callback)
        self.pub = rospy.Publisher("state_estimate", StateArray)


    def depth0_callback(self, data):
        self.depths[0] = data.data

    def depth1_callback(self, data):
        self.depths[1] = data.data

    def depth2_callback(self, data):
        self.depths[2] = data.data

    def depth3_callback(self, data):
        self.depths[3] = data.data
    
    def planar_callback(self, data):
        t = float(data.header.stamp.secs) + float(data.header.stamp.nsecs)*pow(10.0,-9)
        state_array = StateArray()
        for i, pose in enumerate(data.poses):
            
            # If robot detected
            if (pose.orientation.w == 1.0) and self.depths[i] is not None:
                pose.position.z = self.depths[i]
                z = self._pose_to_z_array(pose)
                # Initialize ekf for robot if necessary
                if self.ekfs[i] is None:
                    state_estimate = np.array([z[0], z[1], z[2], 0., 0., m.sin(z[3]), m.cos(z[3]), 0.])
                    self.ekfs[i] = ekf(t, state_estimate, self.init_P,
                                       self.init_u, self.beluga.f, self.beluga.h, 
                                       self.beluga.F, self.beluga.G, self.beluga.H,
                                       self.beluga.L, self.Q, self.R)
                else:
                    state_estimate = self.ekfs[i].estimate(t, z);

            # On no measurement
            else:
                # If estimator initialized, just propagate state
                if self.ekfs[i] is not None:
                    state_estimate = self.ekfs[i].look_forward(t)
                # Otherwise  estimate is zeros.
                else:
                    state_estimate = np.zeros(8)
                    state_estimate[6] = 1.0
                    
            state = self._x_array_to_state(state_estimate)

            #Signify no estimate (due to tracking not started)
            if pose.orientation.w is not 1 and self.ekfs[i] is None:
                state.pose.orientation.w = 0
            else:
                state.pose.orientation.w = 1

            state_array.states.append(state)
        #Pass time to state message
        state_array.header.stamp = data.header.stamp
        self.pub.publish(state_array)
        

    def input_callback(self, data):
        t = float(data.header.stamp.secs) + float(data.header.stamp.nsecs)*pow(10.,-9)
        for i, beluga_input in enumerate(data.belugas):
            u = self._BelugaInput_to_u_array(beluga_input)
            if self.ekfs[i] is not None:
                self.ekfs[i].update_u(t, u)
            
    def publish_estimate(self):

        state_array = StateArray()
        now = rospy.get_rostime()
        t = float(now.secs) + float(now.nsecs)*pow(10,-9)
        for i, estimator in enumerate(self.ekfs):
            # If not initialized
            if estimator is None:
                state_estimate = np.zeros(7)
                state = self._x_array_to_state(state_estimate)
                state.pose.orientation.w = 0
            else:
                state_estimate = estimator.look_forward(t)
                state = self._x_array_to_state(state_estimate)
                state.pose.orientation.w = 1
            state_array.states.append(state)
            
        state_array.header.stamp = now
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
    def _x_array_to_state(self, x):
        state = State()
        state.pose.position.x = x[0]
        state.pose.position.y = x[1]
        state.pose.position.z = x[2]
        state.pose.orientation.z = m.atan2(x[5],x[6])
        state.twist.linear.x = x[3]
        state.twist.linear.z = x[4]
        state.twist.angular.z = x[6]
        return state
    
    ##
    #
    #
    def _BelugaInput_to_u_array(self, belugaInput):
        u[0] = belugaInput.thrust_motor
        u[1] = belugaInput.servo
        u[2] = belugaInput.vertical_motor
        return u
    
def main():
    rospy.init_node('beluga_estimator')

    estimator = BelugaEstimator()
    
    r = rospy.Rate(15) # Make this set dynamically or by param
    while not rospy.is_shutdown():
        estimator.publish_estimate()
        r.sleep()


if __name__ == '__main__':
    main()
