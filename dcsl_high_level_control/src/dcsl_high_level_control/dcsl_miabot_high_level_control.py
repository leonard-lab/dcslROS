#!/usr/bin/env python

## @file
#
#
#

## @author Will Scott

import numpy as np
import math as m
import sys
import imp

import rospy
from geometry_msgs.msg import PoseArray, Twist, Pose
from dcsl_messages.msg import TwistArray, StateArray, State
from dynamic_reconfigure.server import Server
from dcsl_high_level_control.cfg import dcsl_miabot_high_level_control_configConfig as Config
from miabot_logger import MiabotLogger
##
#
#
class MiabotHighLevelController(object):
    
    ##
    #
    #
    def __init__(self, n_robots, control_law):

        self.going_to_initial_conds = True

        self.n_robots = int(n_robots)
        self.control_law = control_law
        self.control_type = self.control_law.control_type()
        # 1 = velocity control law, 2 = waypoint control law
        self.initial_positions = self.control_law.get_initial_conds()

        self.mode = 1
        self.state_data = None
        self.state_array = None
        self.last_pub = -10000000

        # Get parameters for waypoint controller
        self.min_dist = rospy.get_param("waypoint_tolerance", 0.02)

        # Set up dynamic reconfigure for control modes
        self.serv = Server(Config, self.reconfigure_callback)
        self.run_initial = False
        self.run_control = False


        # Subscribe to current state information
        self.estimate_sub = rospy.Subscriber("state_estimate", StateArray, self.estimate_callback, queue_size = 1)

        # Set up publishers for high level control inputs
        self.velocity_pub = rospy.Publisher("velocity_input", TwistArray)
        self.wp_pub = rospy.Publisher("wp_input", PoseArray)
        
        # Create name for logger
        self.logger = None
    ##
    #
    #
    def reconfigure_callback(self, config, level):
        self.run_initial = bool(config["go_to_initial_states"])
        self.run_control = bool(config["run_control_law"])
        return config

    ##
    #
    #
    def estimate_callback(self, data):
        self.state_data = data

        if self.state_data is not None:
            t = float(self.state_data.header.stamp.secs) + float(self.state_data.header.stamp.nsecs)*pow(10.0, -9)
            self.state_array = self._states_message_to_array(self.state_data.states)

            if self.going_to_initial_conds and self.run_initial:
                if self._close_enough_to_wp(self.state_array,self.initial_positions,self.min_dist):
                    self.going_to_initial_conds = False            
                else:
                    # else publish waypoint message
                    self.wp_pub.publish(self._wp_array_to_message(t, self.initial_positions))
                
            elif self.run_control and self.control_type == 1:
                # call control law to calculate velocity inputs
                self.vel_array = self.control_law.velocity_controller(t, self.state_array)
                self.velocity_pub.publish(self._vel_array_to_message(t, self.vel_array))
                if not self.logger:
                    self.logger = MiabotLogger(self.n_robots, self.control_type)
                self.logger.add_to_log(t, self.state_array, self.vel_array)

            elif self.run_control and self.control_type == 2:
                # call control law to calculate waypoint inputs
                self.wp_array = self.control_law.waypoint_controller(t, self.state_array)
                self.wp_pub.publish(self._wp_array_to_message(t, self.wp_array))
                if not self.logger:
                    self.logger = MiabotLogger(self.n_robots, self.control_type)
                self.logger.add_to_log(t, self.state_array, self.wp_array)
            else:
                # send zero inputs to stop robots
                self.velocity_pub.publish(self._vel_array_to_message(t, np.zeros([self.n_robots,3])))

        else:
            # send zero inputs to stop robots
            self.velocity_pub.publish(self._vel_array_to_message(t, np.zeros([self.n_robots,3])))

        self.last_pub = rospy.get_time()

        # export and destroy logger object if no longer running control
        if self.logger and not self.run_control:
            # check if control law contains parameter info function
            parameter_info = getattr(self.control_law, "parameter_info", None)
            if callable(parameter_info):
                logfile = self.logger.export_to_mat(self.control_law.parameter_info())
            else:
                logfile = self.logger.export_to_mat()
            rospy.loginfo('state trajectory saved to ' + logfile)
            self.logger = None

    ## 
    #
    #
    def _states_message_to_array(self, states):
        x = np.empty([self.n_robots, 3])
        for i, state in enumerate(states):
            x[i,0] = state.pose.position.x
            x[i,1] = state.pose.position.y
            x[i,2] = state.pose.orientation.z
        return x

    ## 
    #
    #
    def _wp_array_to_message(self, t, wp):
        waypoint_message = PoseArray()
        waypoint_message.header.stamp = rospy.get_rostime()
        for i in range(0,self.n_robots):
            p = Pose()
            p.position.x = wp[i,0]
            p.position.y = wp[i,1]
            p.orientation.z = wp[i,2]
            waypoint_message.poses.append(p)
        return waypoint_message

    ## 
    #
    #
    def _vel_array_to_message(self, t, u):
        vel_message = TwistArray()
        vel_message.header.stamp = rospy.get_rostime()
        for i in range(0, self.n_robots):
            vel = Twist()
            vel.linear.x = u[i,0]
            vel.angular.z = u[i,1]
            vel_message.twists.append(vel)
        return vel_message

    ## 
    #
    #
    def _close_enough_to_wp(self, x, wp, tol):
        # check if total error from x to wp is less than tol
        absdiff = np.absolute(x-wp)
        for i in range(0,self.n_robots):
            if absdiff[i,2] > m.pi:
                absdiff[i,2] = abs(absdiff[i,2]-2*m.pi)
        return np.sum(absdiff) < tol



def main():
    rospy.init_node("miabot_high_level_controller")

    # command line args should be 'n_robots /path/to/control/law'

    n_robots = int(sys.argv[1])

    control_law_path = sys.argv[2]
    
    control_law_module = imp.load_source('control_law_module', control_law_path)

    control_law = control_law_module.MiabotControlLaw(n_robots)
    controller = MiabotHighLevelController(n_robots, control_law)
    
    rospy.spin()

if __name__ == '__main__':
    main()
