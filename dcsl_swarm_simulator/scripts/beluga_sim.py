#!/usr/bin/env python

## @file
# This node simulates the behavior of a fleet of Beluga robots. It subscribes to the command array and integrates the states of the robots over time. It publishes simulated planar measurements and depth measures at a regular interval.
# This node is meant to be used in place of the Beluga driver, swarm director, and vision system so that the behavior of the system may be simulated with the ROS communications and estimator without using the real robots.
# Usage beluga_sim.py [n_robots]

## @author Brendan Andrade

from scipy.integrate import odeint
import numpy as np
import math as m
import sys

import rospy
from geometry_msgs.msg import PoseArray, Pose
from dcsl_messages.msg import BelugaArray
from std_msgs.msg import Float32

from dcsl_beluga_main.beluga import Beluga # Import the dynamic system equations stored in the main package.

###############################################

## This class defines a ROS node for simulating the dynamics of a group of Beluga robots.
#
# It subscribes to the cmd_inputs topic containing BelugaArray messages with commands for the robots.
# It publishes to the robotN/depth_measurement topics with Float32 messages with the depth of each robot.
# It publishes to the /planar_measurements topic with PoseArray messages of the X,Y,theta poses of each robot.
# It takes n_robots as an argument.
class BelugaSimulator(object):

    ## On initialization, the pubs and subs are created and the robots are initialized to their nominal real start positions in the tank.
    #
    # @param n_robots is an integer number of robots to simulate. Up to 4 currently.
    def __init__(self, n_robots):
        
        init1 = np.array((0., -1., 1.75, 0., 0., 0., 1., 0.))
        init2 = np.array((-1., 0., 1.75, 0., 0., 0., 1., 0.))
        init3 = np.array((-1., .5, 1.75, 0., 0., 0., 1., 0.))
        init4 = np.array((1., 0., 1.75, 0., 0., 0., 1., 0.))
        initial_poses = np.array((init1, init2, init3, init4))
        
        self._n_robots = n_robots
        model = Beluga()
        self._sys_func = lambda x, t, u: model.f(x, u, t)
        self._input_sub = rospy.Subscriber('/cmd_inputs', BelugaArray, self.input_callback)
        self._planar_pub = rospy.Publisher('/planar_measurements', PoseArray)
        self._depth_pub_array = []
        self._state_array = []
        for i in xrange(0, self._n_robots):
            name = "robot" + str(i) + "/depth_measurement"
            pub = rospy.Publisher(name, Float32)
            self._depth_pub_array.append(pub)
            self._state_array.append(initial_poses[i])
        now = rospy.get_rostime()
        self._t = float(now.secs) + float(now.nsecs)*pow(10., -9)
        self._current_input = np.zeros((self.n_robots, 3))

    ## Callback function for command inputs. Integrates to advance the system to the current time.
    #
    # @param data is the message data received by the subscriber.
    def input_callback(self, data):
        # Get ROS time
        now = rospy.get_rostime()
        new_t = float(now.secs) + float(now.nsecs)*pow(10., -9)
        t_array = np.array([self._t, new_t])
        for i in xrange(0, self._n_robots):
            # Advance states to current time with old inputs
            y = odeint(self._sys_func, self._state_array[i], t_array, args=(self._current_input[i],))
            self._state_array[i] = y[-1]
            # Update current input commands
            self._current_input[i] = np.array((data.belugas[i].thrust_motor, data.belugas[i].servo, data.belugas[i].vertical_motor))
        # Update current time
        self._t = new_t
        
    ## Integrates the states of the robots to the current time and publishes the simulated sesnsor topics.
    def publish_state(self):
        # Get ROS time
        now = rospy.get_rostime()
        new_t = float(now.secs) + float(now.nsecs)*pow(10.,-9)
        t_array = np.array([self._t, new_t])
        pose_array = PoseArray()
        pose_array.header.stamp = now
        for i in xrange(0, self._n_robots):
            # Advance states to current time
            y = odeint(self._sys_func, self._state_array[i], t_array, args=(self._current_input[i],))
            self._state_array[i] = y[-1]
            # Create planar measurement pose array
            pose = Pose()
            pose.position.x = self._state_array[i][0]
            pose.position.y = self._state_array[i][1]
            pose.position.z = self._state_array[i][2]
            pose.orientation.z = m.atan2(self._state_array[i][5], self._state_array[i][6])
            pose.orientation.w = 1.0
            pose_array.poses.append(pose)
            #Publish depth measurment
            self._depth_pub_array[i].publish(self._state_array[i][2])
        #Publish pose_array
        self._planar_pub.publish(pose_array)
        # Update current time
        self._t = new_t

## Starts the node and then enters a loop to publish simulated sensor messages.
def main():
    rospy.init_node('beluga_swarm_simulator')

    n_robots = int(sys.argv[1])

    simulator = BelugaSimulator(n_robots)
    
    r = rospy.Rate(55)
    while not rospy.is_shutdown():
        simulator.publish_state()
        r.sleep()

if __name__ == '__main__':
    main()
