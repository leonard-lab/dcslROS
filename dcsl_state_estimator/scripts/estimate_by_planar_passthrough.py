#!/usr/bin/env python


import roslib
roslib.load_manifest('dcsl_state_estimator')
import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Float32

class PassthroughEstimator:

    def __init__(self):
        self.depth_sub = rospy.Subscriber("depth_meters", Float32, self.depth_callback)
        self.planar_sub = rospy.Subscriber("planar_measurements", PoseArray, self.planar_callback)
        self.pub = rospy.Publisher("state_estimate", PoseArray)

    def depth_callback(self, data):
        self.depth = data.data
        

    def planar_callback(self, data):
        if data.poses[0].orientation.w == 1:
            measured_pose = data.poses[0]
            measured_pose.position.z = self.depth
            array = PoseArray()
            array.poses.append(measured_pose)
            self.pub.publish(array)

def main():
    rospy.init_node('passthrough_estimator')
    estimator = PassthroughEstimator()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"

if __name__ == '__main__':
    main()
