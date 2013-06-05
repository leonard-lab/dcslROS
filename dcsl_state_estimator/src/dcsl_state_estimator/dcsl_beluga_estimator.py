#!/usr/bin/env python

## @file
# Estimator node for Beluga UUV.
#
#

## @author Brendan Andrade

from ukf_API import ukf

import numpy as np
import math as m
import sys

import rospy
from geometry_msgs.msg import PoseArray, Pose
from geometry_msgs.msg import Twist
from dcsl_messages.msg import TwistArray
from std_msgs.msg import Int16

from dcsl_beluga_main.beluga import Beluga

###############################################

##
class BelugaEstimator(object):
    
    ##
    def __init__(self, n_robots):
        
        self.n = n_robots

        self.beluga = Beluga()
        Q = np.eye(7, dtype=float)
        R = np.eye(4, dtype=float)
        x_init = np.zeros(7)
        self.ukfs = []
        self.Ts = 0.01
        self.time = 0.0
        self.time_step = 0
        for i in range(0, self.n):
            self.ukfs.append(ukf(self.beluga.f, self.beluga.g, Q, R, x_init, 7, 3, 4, self.Ts))


        depth_callback_list = [self.depth0_callback, self.depth1_callback, self.depth2_callback, self.depth3_callback]
        self.depth_sub_array = []
        for i in range(0, self.n):
            name = "robot" + str(i) +"/depth"
            subscriber = rospy.Subscriber(name, Int16, depth_callback_list[i])
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
        new_time = data.header.stamp.secs + data.header.stamp.nsecs*pow(10.0,-9) - self.start_time
        new_time_step = int(new_time/self.Ts)
        
        pose_array = PoseArray()
        
        for i in range(0, self.n):
            if new_time_step - self.time_step > 2:
                Y = np.ma.array([0,0,0,0], mask=[1,1,1,1])
                U = self.last_cmds[i]
                for j in range(self.time_step+1, new_time_step):
                    self.ukfs[i].update(Y, U)
            
            if data.poses[i].orientation.w == 1:
                Y = np.array([data.poses[i].position.x, data.poses[i].position.y, self.depth[i], data.poses[i].orientation.z])
            else:
                Y = np.ma.array([0,0,0,0], mask=[1,1,1,1])
            
            U = self.cmds[i] #Fix timing of this
            self.ukfs[i].update(Y,U)            
            X = self.ukfs[i].estimate()
            pose = Pose()
            pose.position.x = X[0]
            pose.position.y = X[1]
            pose.position.z = X[2]
            pose.orientation.z = X[5]
            pose_array.poses.append(pose)
            
        pose_array.header.stamp = rospy.Time.now() #Consider passing original time.
        self.pub.publish(pose_array)
            
        self.time = new_time
        self.time_step = new_time_step
        self.last_cmds = self.cmds

    def cmd_callback(self, data):
        for i, twist in enumerate(data.twists):
            self.cmds[i] = np.array([twist.linear.x, twist.angular.x, twist.linear.z])
            
        
    
def main():
    rospy.init_node('beluga_estimator')

    n_robots = int(sys.argv[1])

    estimator = BelugaEstimator(n_robots)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"

if __name__ == '__main__':
    main()
