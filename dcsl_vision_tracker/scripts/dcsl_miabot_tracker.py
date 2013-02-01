#!/usr/bin/env python

## @file
# A node that subscribes to the /camera/image_raw topic with Image messages, uses OpenCV to find the poses of the robots in the images, and publishes the poses of the robots as a PoseArray. It also subscribes to the state_estimate topic (PoseArray messages) so that the published PoseArray maintains the same order as the state_estimate. It matches the detected robots with the estimated poses using the Hungarian algorithm via the munkres library.

## @author Brendan Andrade


import roslib
roslib.load_manifest('dcsl_vision_tracker')
import rospy
from geometry_msgs.msg import PoseArray,Pose
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv

from dcsl_vision_tracker_API import DcslMiabotTracker, DcslPose

## This class allows detection of robots in an image,  orders these readings based on state estimates, and publishes them as a PoseArray.
class miabot_tracker:
    
    ## Creates publishers and subscribers, loads background image and nRobots parameter, and creates CvBridge object.
    def __init__(self):
        self.image_pub = rospy.Publisher("tracked_image",Image)
        self.measurement_pub = rospy.Publisher("planar_measurements", PoseArray)
        self.image_sub = rospy.Subscriber("/camera/image_raw",Image,self.imageCallback)
        self.state_sub = rospy.Subscriber("state_estimate", PoseArray, self.stateCallback)
        self.bridge = CvBridge()

        location = rospy.get_param('/vision_tracker/background_image')
        background = cv.LoadImageM(location,cv.CV_LOAD_IMAGE_GRAYSCALE)
        threshold = 100
        erode_iterations = 1
        min_blob_size = 500
        max_blob_size = 2000
        scale = 1.7526/1024.0 #meters/pixel
        image_width = 1024
        image_height = 768
        self.storage = cv.CreateMemStorage()
        self.tracker = DcslMiabotTracker(background, threshold, erode_iterations, min_blob_size, max_blob_size, scale, self.storage, image_width, image_height)

        '''
        #For testing
        temp1 = DcslPose()
        temp1.set_position((0,0,0))
        temp1.set_quaternion((0,0,0,0))
        temp2 = DcslPose()
        temp2.set_position((-1,-1,0))
        temp2.set_quaternion((0,0,0,0))
        self.current_states = [temp1,temp2]
        '''

    ## Callback function for when new images are received. Senses positions of robots, sorts them into the correct order, publishes readings, and displays image.
    #
    # @param data is the image data received from the /camera/image_raw topic
    def imageCallback(self,data):
        # Bridge image from ROS message to OpenCV
        try:
            working_image = self.bridge.imgmsg_to_cv(data, "mono8")
        except CvBridgeError, e:
            print e

        # Create BGR8 image to be output
        output_image = cv.CreateMat(working_image.height,working_image.width,cv.CV_8UC3)
        cv.Merge(working_image,working_image,working_image, None, output_image)        
        
        # Find poses and place into a message
        measurements = self.tracker.get_poses(working_image, self.current_states, 0)
        pose_array = PoseArray()
        for dcsl_pose in measurements:
            p = Pose()
            if dcsl_pose.detected:
                p.position.x = dcsl_pose.position_x()
                p.position.y = dcsl_pose.position_y()
                p.orientation.z = dcsl_pose.quaternion_z()
                p.orientation.w = 1
            else:
                p.orientation.w = 0
            pose_array.poses.append(p)

        try:
            self.image_pub.publish(self.bridge.cv_to_imgmsg(output_image,"bgr8"))
        except CvBridgeError, e:
            print e

        self.measurement_pub.publish(pose_array)

    ## Callback function for state estimates. Stores state estimates as self.currentStates.
    #
    # @param data is the message received from the subscriber.
    def stateCallback(self, data):
        self.current_states = []
        for pose in data.poses:
            temp = DcslPose()
            pos_x = pose.position.x
            pos_y = pose.position.y
            theta = pose.orientation.z
            temp.set_position((pos_x,pos_y,0))
            temp.set_quaternion((0,0,theta,0))
            self.current_states.append(temp)

## Runs on the startup of the node. Initializes the node and creates the MiabotTracker object.
def main():
    rospy.init_node('dcsl_miabot_tracker')
    tracker = miabot_tracker()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
        cv.DestroyAllWindows()


if __name__ == '__main__':
    main()

# http://www.ros.org/wiki/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython
