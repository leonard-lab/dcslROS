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
from dynamic_reconfigure.server import Server
from dcsl_low_level_control.cfg import dcsl_beluga_low_level_control_configConfig as Config

from dcsl_low_level_control.beluga_velocity_controller import BelugaVelocityController

##
#
#
class BelugaLowLevelController(object):
    
    ##
    #
    #
    def __init__(self, n_robots):

        self.n_robots = int(n_robots)

        self.mode = None
        self.current_commands = None
        self.state_data = None
        self.last_pub = -10000000

        # Get parameters
        vc_axis_1 = np.array(rospy.get_param('~axis_1_coordinates'))
        vc_axis_2 = np.array(rospy.get_param('~axis_2_coordinates'))
        vc_axis_3 = np.array(rospy.get_param('~axis_3_coordinates'))
        vc_axis_4 = np.array(rospy.get_param('~axis_4_coordinates'))
        #Ki = np.array(rospy.get_param('~Ki'))
        K_shape = tuple(rospy.get_param('~K_shape'))
        K = np.array(rospy.get_param('~K_flat')).reshape(K_shape, order='F') #MATLAB uses fortran order

        # Define velocity control law
        self.velocity_controller = BelugaVelocityController(K, vc_axis_1, vc_axis_2, vc_axis_3, vc_axis_4)
        self.velocity_control_law = self.velocity_controller.control_law

        # Setup publisher for commands to belugas
        self.input_pub = rospy.Publisher("cmd_inputs", BelugaArray)

        # Setup dynamic reconfigure server to give joystick selections
        self.srv = Server(Config, self.joystick_selection_callback)
        self.active_joy = -1
        self.joy_command = None

        # Subscribe to current state information
        self.estimate_sub = rospy.Subscriber("state_estimate", StateArray, self.estimate_callback, queue_size = 1)

        # Subscribe to possible high level input streams
        self.direct_sub = rospy.Subscriber("direct_input", BelugaArray, self.direct_callback)
        self.velocity_sub = rospy.Subscriber("velocity_input", TwistArray, self.velocity_callback)
        self.wp_sub = rospy.Subscriber("wp_input", PoseArray, self.wp_callback)
        
        # Subscribe to joystick
        self.joystick_sub = rospy.Subscriber("joy_cmd", belugaInput, self.joy_callback)

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
    def direct_callback(self, data):
        self.mode = 0
        self.current_commands = data
        self.publish_inputs()

    ##
    #
    #
    def velocity_callback(self, data):
        self.mode = 1
        commands = np.zeros((self.n_robots, 3))
        for i in xrange(0, self.n_robots):
            commands[i] = np.array([data.twists[i].linear.x, data.twists[i].linear.z, data.twists[i].angular.z])
        self.current_commands = commands
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
        if self.mode == 0:
            beluga_input_array = self.current_commands
        elif self.state_data is not None:
            beluga_input_array = BelugaArray()
            # Consider service call here to get most recent state data
            t = float(self.state_data.header.stamp.secs) + float(self.state_data.header.stamp.nsecs)*pow(10.0, -9)
            for i, state in enumerate(self.state_data.states):
                x = self._state_to_mat(state)
                if self.mode == 1:
                    u_direct = self.velocity_control_law(x, np.zeros(3), self.current_commands[i])
                elif self.mode == 2:
                    u_direct = self.wp_controller(x, self.current_commands)
                else:
                    u_direct = np.array([0., 0., 0.])
                beluga_input = self._u_direct_to_beluga_input(u_direct)
                beluga_input_array.belugas.append(beluga_input)
        else:
            beluga_input_array = BelugaArray()
            for i in xrange(0, self.n_robots):
                u_direct = np.array([0., 0., 0.])
                beluga_input = self._u_direct_to_beluga_input(u_direct)
                beluga_input_array.belugas.append(beluga_input)
        # Override any supplied control with joystick
        if self.active_joy >= 0 and self.active_joy < self.n_robots and self.joy_command is not None:
            beluga_input_array.belugas[self.active_joy] = self.joy_command
        # Publish
        beluga_input_array.header.stamp = rospy.get_rostime()
        self.input_pub.publish(beluga_input_array)
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
    def _u_direct_to_beluga_input(self, u):
        b = belugaInput()
        b.thrust_motor = u[0]
        b.servo = u[1]
        b.vertical_motor = u[2]
        return b

def main():
    rospy.init_node("beluga_low_level_controller")

    n_robots = sys.argv[1]

    controller = BelugaLowLevelController(n_robots)
    
    rospy.spin()

if __name__ == '__main__':
    main()
