#!/usr/bin/env python

## @file
#
#
#

## @author Brendan Andrade

import numpy as np
import math as m
import sys

import rospy
from geometry_msgs.msg import PoseArray
from dcsl_messages.msg import TwistArray, StateArray, State, BelugaArray, belugaInput

##
#
#
class BelugaLowLevelController(object):
    
    ##
    #
    #
    def __init__(self):

        self.mode = None
        self.current_commands = None

        # Setup publisher for commands to belugas
        self.input_pub = rospy.Publisher("cmd_inputs", BelugaArray)

        # Subscribe to current state information
        self.estimate_sub = rospy.Subscriber("state_estimate", StateArray, self.estimate_callback)

        # Subscribe to possible high level input streams
        self.direct_sub = rospy.Subscriber("direct_input", BelugaArray, self.direct_callback)
        self.velocity_sub = rospy.Subscriber("velocity_input", TwistArray, self.velocity_callback)
        self.wp_sub = rospy.Subscriber("wp_input", PoseArray, self.wp_callback)

    ##
    #
    #
    def direct_callback(self, data):
        self.mode = 0
        self.current_commands = data
    
    ##
    #
    #
    def velocity_callback(self, data):
        self.mode = 1
        self.current_commands = data

    ##
    #
    #
    def wp_callback(self, data):
        self.mode = 2
        u_wp = []
        for pose in data.poses:
            wp = np.zeros(4)
            wp[0] = pose.position.x
            wp[1] = pose.position.y
            wp[2] = pose.position.z
            wp[3] = pose.orientation.z
            u_wp.append(wp)
        self.current_commands = u_wp

    ##
    #
    #
    def estimate_callback(self, data):
        if self.mode == 0:
            self.input_pub.publish(self.current_commands)
        elif self.mode == 1 or self.mode == 2:
            direct_array = BelugaArray()
            for i, state in enumerate(data.states):
                x = self._state_to_mat(state)
                if self.mode == 1:
                    u_direct = self.velocity_controller(x, self.current_commands)
                elif self.mode == 2:
                    u_direct = self.wp_controller(x, self.current_commands)
                beluga_input = self._u_direct_to_beluga_input(u_direct)
                direct_array.append(beluga_input)
            direct_array.header = rospy.get_time()
            self.input_pub.publish(direct_array)
                
    ## 
    #
    #
    def _state_to_mat(self, state):
        x = np.zeros(8)
        x[0] = state.pose.position.x
        x[1] = state.pose.position.y
        x[2] = state.pose.position.z
        x[3] = state.twist.linear.x
        x[4] = state.twist.linear.z
        x[5] = m.sin(state.pose.orientation.z)
        x[6] = m.cos(state.pose.orientation.z)
        x[7] = state.twist.angular.z
        return x

    ##
    #
    #
    def _u_direct_to_beluga_input(u):
        b = belugaInput
        b.thrust_motor = u[0]
        b.servo = u[1]
        b.vertical_motor = u[2]
        return b

def main():
    rospy.init_node("beluga_low_level_controller")
    
    controller = BelugaLowLevelController()
    
    rospy.spin()

if __name__ == '__main__':
    main()
