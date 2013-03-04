#!/usr/bin/env python

## @file
# A node that subscribes to the /camera/image_raw topic with Image messages, uses OpenCV to find the poses of the Miabot robots in the images, and publishes the poses of the robots as a PoseArray. It also subscribes to the state_estimate topic (PoseArray messages) so that the published PoseArray maintains the same order as the state_estimate. It matches the detected robots with the estimated poses using the Hungarian algorithm via the munkres library.

## @author Brendan Andrade


import roslib
roslib.load_manifest('dcsl_vision_tracker')
import rospy
from geometry_msgs.msg import PoseArray,Pose
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from dynamic_reconfigure.server import Server
from dcsl_vision_tracker.cfg import dcsl_miabot_tracker_configConfig as Config

import cv
import math as m



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
        self.srv = Server(Config, self.parameter_callback)

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
        self.tracker = DcslMiabotTracker([background], None, threshold, erode_iterations, min_blob_size, max_blob_size, self.storage, image_width, image_height, scale)

    
        # For testing
        temp1 = DcslPose()
        temp1.set_position((0,0,0))
        temp1.set_quaternion((0,0,0,0))
        temp2 = DcslPose()
        temp2.set_position((-1,-1,0))
        temp2.set_quaternion((0,0,0,0))
        self.current_states = [temp1,temp2]
        

    ## Callback function for when new images are received. Senses positions of robots, sorts them into the correct order, publishes readings, and displays image.
    #
    # @param data is the image data received from the /camera/image_raw topic
    def imageCallback(self,data):
        # Bridge image from ROS message to OpenCV
        try:
            working_image = self.bridge.imgmsg_to_cv(data, "mono8")
        except CvBridgeError, e:
            print e
        
        
        # Find poses and place into a message
        measurements, image_poses, contours = self.tracker.get_poses(working_image, self.current_states, 0)
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

        #Publish measuremented poses
        self.measurement_pub.publish(pose_array)

        # Create BGR8 image to be output
        output_image = cv.CreateMat(working_image.height,working_image.width,cv.CV_8UC3)
        cv.Merge(working_image,working_image,working_image, None, output_image)

        # Overlay position, heading, and contour
        length = 15.0
        radius = 5
        cyan = cv.RGB(0,255,255)
        for point in image_poses:
            if point.detected:
                end = (int(point.position_x()+m.cos(point.quaternion_z())*length),int(point.position_y()-m.sin(point.quaternion_z())*length))
                center = (int(point.position_x()), int(point.position_y()))
                cv.Line(output_image, center, end, cyan)
                cv.Circle(output_image, center, radius, cyan)
        cv.DrawContours(output_image, contours, cyan, cyan, 2)
        

        # Publish image with overlay
        try:
            self.image_pub.publish(self.bridge.cv_to_imgmsg(output_image,"bgr8"))
        except CvBridgeError, e:
            print e        

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

    ## Call back function for parameter updates.
    #
    # @param config data received for parameter update
    # @param level
    def parameter_callback(self, config, level):
        if "tracker" in self.__dict__:
            self.tracker.threshold = config["binary_threshold"]
            self.tracker.erode_iterations = config["erode_iterations"]
            self.tracker.min_blob_size = config["min_blob_size"]
            self.tracker.max_blob_size = config["max_blob_size"]
            self.tracker.scale = config["scale"]
            location = config["background_file"]
            background = cv.LoadImageM(location,cv.CV_LOAD_IMAGE_GRAYSCALE)
            self.background_list = [background]
        rospy.logdebug("""Reconfigure Request: {background_file}, {binary_threshold}, {erode_iterations}, {min_blob_size}, {max_blob_size}, {scale}""".format(**config))
        return config
    

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
