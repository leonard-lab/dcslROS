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
from geometry_msgs.msg import PoseArray, Twist
from dcsl_messages.msg import TwistArray, StateArray, State
from dynamic_reconfigure.server import Server
from dcsl_low_level_control.cfg import dcsl_miabot_low_level_control_configConfig as Config

##
#
#
class MiabotLowLevelController(object):
    
    ##
    #
    #
    def __init__(self, n_robots):

        self.n_robots = int(n_robots)

        self.mode = None
        self.current_commands = None
        self.state_data = None
        self.last_pub = -10000000

        # Get parameters for waypoint controller
        self.k1 = rospy.get_param("waypoint_gain_1", 0.7)
        self.k2 = rospy.get_param("waypoint_gain_2", 0.5)
        self.min_dist = rospy.get_param("waypoint_tolerance", 0.02)

        # Setup publisher for commands to belugas
        self.input_pub = rospy.Publisher("cmd_vel_array", TwistArray)

        # Setup dynamic reconfigure server to give joystick selections
        self.srv = Server(Config, self.joystick_selection_callback)
        self.active_joy = -1
        self.joy_command = None

        # Subscribe to current state information
        self.estimate_sub = rospy.Subscriber("state_estimate", StateArray, self.estimate_callback, queue_size = 1)

        # Subscribe to possible high level input streams
        self.velocity_sub = rospy.Subscriber("velocity_input", TwistArray, self.velocity_callback)
        self.wp_sub = rospy.Subscriber("wp_input", PoseArray, self.wp_callback)
        
        # Subscribe to joystick
        self.joystick_sub = rospy.Subscriber("joy_cmd", Twist, self.joy_callback)

    ##
    #
    #
    def joystick_selection_callback(self, config, level):
        self.active_joy = int(config["JoystickRobot"])
        return config
    ##
    #
    #
    def joy_callback(self, data):
        self.joy_command = data

    ##
    #
    #
    def velocity_callback(self, data):
        self.mode = 1
        self.current_commands = data
        self.publish_inputs()

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
        self.publish_inputs()

    ##
    #
    #
    def estimate_callback(self, data):
        self.state_data = data
        if rospy.get_time() > self.last_pub+0.05:
            self.publish_inputs()
    
    ##
    #
    #
    def publish_inputs(self):
        if self.mode == 1:
            miabot_input_array = self.current_commands

        elif self.state_data is not None and self.mode == 2:
            miabot_input_array = TwistArray()
            # Consider service call here to get most recent state data
            t = float(self.state_data.header.stamp.secs) + float(self.state_data.header.stamp.nsecs)*pow(10.0, -9)
            for i, state in enumerate(self.state_data.states):
                x = self._state_to_mat(state)
                u_direct = self._wp_controller(x, self.current_commands[i])
                miabot_twist = self._u_direct_to_twist(u_direct)
                miabot_input_array.twists.append(miabot_twist)
        else:
            miabot_input_array = TwistArray()
            for i in xrange(0, self.n_robots):
                u_direct = np.array([0., 0., 0.])
                miabot_twist = self._u_direct_to_twist(u_direct)
                miabot_input_array.twists.append(miabot_twist)
        # Override any supplied control with joystick
        if self.active_joy >= 0 and self.active_joy < self.n_robots and self.joy_command is not None:
            miabot_input_array.twists[self.active_joy] = self.joy_command
        # Publish
        miabot_input_array.header.stamp = rospy.get_rostime()
        self.input_pub.publish(miabot_input_array)
        self.last_pub = rospy.get_time()
            
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
    def _u_direct_to_twist(self, u):
        t = Twist()
        t.linear.x = u[0]
        t.angular.z = u[1]
        t.linear.z = u[2]
        return t

    ##
    #
    #
    def _wp_controller(self, x, wp):
        theta = m.atan2(x[5],x[6])
        r = m.sqrt(pow(x[0]-wp[0],2) + pow(x[1]-wp[1],2))
        phi = m.atan2(wp[1]-x[1], wp[0]-x[0]) - theta

        u = np.zeros(3)
        u[0] = self.k1 * r * m.cos(phi)
        
        if r > self.min_dist:
            if phi <= m.pi*0.5 and phi > -m.pi*0.5:
                u[1] = self.k2*m.sin(phi)
            else:
                u[1] = -self.k2*m.sin(phi)
        else:
            u[1] = -self.k2*m.sin(theta-wp[3])
 
        return u


def main():
    rospy.init_node("miabot_low_level_controller")

    n_robots = sys.argv[1]

    controller = MiabotLowLevelController(n_robots)
    
    rospy.spin()

if __name__ == '__main__':
    main()
