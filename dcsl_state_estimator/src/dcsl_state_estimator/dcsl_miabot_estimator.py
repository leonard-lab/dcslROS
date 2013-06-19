#!/usr/bin/env python

## @file
#

## @author Brendan Andrade

import rospy
from geometry_msgs.msg import PoseArray, Pose
from dcsl_messages.msg import TwistArray, StateArray
from dcsl_state_estimator.ekf_API import ekf
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
        
        self.ekf = ekf(
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
    
    
def main():
    rospy.init_node('dcsl_miabot_estimator')
    estimator = MiabotEstimator()
    try:
        rospy.spin()
    except KeyboardInterrupt: # Consider switching to on_shutdown
        print "Shutting down"

if __name__ == '__main__':
    main()
