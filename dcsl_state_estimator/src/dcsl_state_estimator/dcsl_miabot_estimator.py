#!/usr/bin/env python

## @file
#

## @author Brendan Andrade

import rospy
from geometry_msgs.msg import PoseArray
from dcsl_messages.msg import TwistArray, StateArray, State
from dcsl_state_estimator.ekf_API import ekf

import numpy as np

##
#
#
class MiabotEstimator:
    
    ##
    #
    #
    def __init__(self):
        self.measurement_sub = rospy.Subscriber("planar_measurements", PoseArray, self.measurement_callback)
        self.input_sub = rospy.Subscriber("cmd_vel_array", TwistArray, self.input_callback)
        self.pub = rospy.Publisher("state_estimate", StateArray)
        
        self.ekfs = None

    ##
    #
    #
    def measurement_sub(self, data):
        pass

    ##
    #
    #
    def input_callback(self, data):
        pass

    ##
    #
    #
    def _pose_to_z_array(self, pose):
        z = np.zeros(4)
        z[0] = pose.position.x
        z[1] = pose.position.y
        z[2] = pose.position.z
        z[3] = pose.orientation.z
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
    rospy.init_node('dcsl_miabot_estimator')
    estimator = MiabotEstimator()
    try:
        rospy.spin()
    except KeyboardInterrupt: # Consider switching to on_shutdown
        print "Shutting down"

if __name__ == '__main__':
    main()
