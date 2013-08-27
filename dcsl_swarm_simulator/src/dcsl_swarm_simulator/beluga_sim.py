#!/usr/bin/env python

## @file
#

## @author Brendan Andrade

from scipy.integrate import odeint
import numpy as np
import math as m
import sys

import rospy
from geometry_msgs.msg import PoseArray, Pose
from dcsl_messages.msg import BelugaArray
from std_msgs.msg import Float32

from dcsl_beluga_main.beluga import Beluga

###############################################

##
class BelugaSimulator(object):

    ##
    def __init__(self, n_robots):
        
        init1 = np.array((0., -1., 1.75, 0., 0., 0., 1., 0.))
        init2 = np.array((-1., 0., 1.75, 0., 0., 0., 1., 0.))
        init3 = np.array((-1., .5, 1.75, 0., 0., 0., 1., 0.))
        init4 = np.array((1., 0., 1.75, 0., 0., 0., 1., 0.))
        initial_poses = np.array((init1, init2, init3, init4))
        
        self.n_robots = n_robots
        model = Beluga()
        self.sys_func = lambda x, t, u: model.f(x, u, t)
        self.input_sub = rospy.Subscriber('/cmd_inputs', BelugaArray, self.input_callback)
        self.planar_pub = rospy.Publisher('/planar_measurements', PoseArray)
        self.depth_pub_array = []
        self.state_array = []
        for i in xrange(0, self.n_robots):
            name = "robot" + str(i) + "/depth_measurement"
            pub = rospy.Publisher(name, Float32)
            self.depth_pub_array.append(pub)
            self.state_array.append(initial_poses[i])
        now = rospy.get_rostime()
        self.t = float(now.secs) + float(now.nsecs)*pow(10., -9)
        self.current_input = np.zeros((self.n_robots, 3))

    ##
    def input_callback(self, data):
        # Get ROS time
        new_t = float(data.header.stamp.secs) + float(data.header.stamp.nsecs)*pow(10., -9)
        t_array = np.array([self.t, new_t])
        for i in xrange(0, self.n_robots):
            # Advance states to current time with old inputs
            y = odeint(self.sys_func, self.state_array[i], t_array, args=(self.current_input[i],))
            self.state_array[i] = y[-1]
            # Update current input commands
            self.current_input[i] = np.array((data.belugas[i].thrust_motor, data.belugas[i].servo, data.belugas[i].vertical_motor))
        # Update current time
        self.t = new_t
        
    ##
    def publish_state(self):
        # Get ROS time
        now = rospy.get_rostime()
        new_t = float(now.secs) + float(now.nsecs)*pow(10.,-9)
        t_array = np.array([self.t, new_t])
        pose_array = PoseArray()
        pose_array.header.stamp = now
        for i in xrange(0, self.n_robots):
            # Advance states to current time
            y = odeint(self.sys_func, self.state_array[i], t_array, args=(self.current_input[i],))
            self.state_array[i] = y[-1]
            # Create planar measurement pose array
            pose = Pose()
            pose.position.x = self.state_array[i][0]
            pose.position.y = self.state_array[i][1]
            pose.position.z = self.state_array[i][2]
            pose.orientation.z = m.atan2(self.state_array[i][5], self.state_array[i][6])
            pose.orientation.w = 1.0
            pose_array.poses.append(pose)
            #Publish depth measurment
            self.depth_pub_array[i].publish(self.state_array[i][2])
        #Publish pose_array
        self.planar_pub.publish(pose_array)
        # Update current time
        self.t = new_t

def main():
    rospy.init_node('beluga_swarm_simulator')

    n_robots = int(sys.argv[1])

    simulator = BelugaSimulator(n_robots)
    
    r = rospy.Rate(50)
    while not rospy.is_shutdown():
        simulator.publish_state()
        r.sleep()

if __name__ == '__main__':
    main()
