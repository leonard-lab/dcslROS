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
from dcsl_messages.msg import TwistArray, StateArray, State
from std_msgs.msg import Float32

from dcsl_beluga_main.beluga import Beluga
from dcsl_state_estimator.ekf_API import ekf

###############################################

##
class BelugaEstimator(object):
    
    ##
    def __init__(self, n_robots):

        self.beluga = Beluga()
        self.Q = np.eye(7, dtype=float)
        self.R = np.eye(4, dtype=float)
        self.ekfs = [None]*4;
        x_init = np.zeros(7)

        self.depth = [0]*4
        depth_callback_list = [self.depth0_callback, self.depth1_callback, self.depth2_callback, self.depth3_callback]
        self.depth_sub_array = []
        for i in range(0, self.n):
            name = "robot" + str(i) +"/depth_measurement"
            subscriber = rospy.Subscriber(name, Float32, depth_callback_list[i])
            self.depth_sub_array.append(subscriber)

        self.cmd_sub = rospy.Subscriber("/cmd_inputs", TwistArray, self.cmd_callback)
        self.cmds = [np.zeros(3)]*self.n
        self.last_cmds = self.cmds

        self.planar_sub = rospy.Subscriber("/planar_measurements", PoseArray, self.planar_callback)
        
        self.pub = rospy.Publisher("state_estimate", PoseArray)

        self.start_time = rospy.get_time()

    def depth0_callback(self, data):
        self.depth[0] = data.data

    def depth1_callback(self, data):
        self.depth[1] = data.data

    def depth2_callback(self, data):
        self.depth[2] = data.data

    def depth3_callback(self, data):
        self.depth[3] = data.data
    
    def planar_callback(self, data):
        t = float(data.header.stamp.secs) + float(data.header.stamp.nsecs)*pow(10.0,-9)
        state_array = StateArray()
        for i, pose in enumerate(data.poses):
            # If robot detected
            if (pose.orientation.w == 1.0):
                z = self._pose_to_z_array(pose)
                # Initialize ekf for robot if necessary
                if self.ekfs[i] is None:
                    state_estimate = np.array([z[0], z[1], z[2], 0., 0., z[3], 0.])
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
                    state_estimate = np.zeros(7)
            
            #Wrap theta angle to pi
            if state_estimate[5] >= m.pi or state_estimate[5] < -m.pi:
                state_estimate[5] = state_estimate[5] - m.floor(state_estimate[5]/(2.*m.pi)+0.5)*2.*m.pi
                    
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
        

    def cmd_callback(self, data):
        for i, twist in enumerate(data.twists):
            self.cmds[i] = np.array([twist.linear.x, twist.angular.x, twist.linear.z])
            
    def publish_estimate():

        state_array = StateArray()

        for i, estimator in self.ekfs:
            # If not initialized
            if estimator is None:
                state_estimate = np.zeros(7)
                

    ##
    #
    #
    def _x_array_to_state(self, x):
        state = State()
        state.pose.position.x = x[0]
        state.pose.position.y = x[1]
        state.pose.position.z = x[2]
        state.pose.orientation.z = x[5]
        state.twist.linear.x = x[3]
        state.twist.linear.z = x[4]
        state.twist.angular.z = x[6]
        return state
    
    
def main():
    rospy.init_node('beluga_estimator')

    n_robots = int(sys.argv[1])

    estimator = BelugaEstimator(n_robots)
    
    
    r = rospy.Rate(15) # Make this set dynamically or by param
    while not rospy.is_shutdown():
        estimator.publish_estimate()
        r.sleep()


if __name__ == '__main__':
    main()
