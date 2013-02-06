#!/usr/bin/env python

## @file
# A node that subscribes to the /cameraX/image_raw topics (where X is the number of the camera) with Image messages, uses OpenCV to find the poses of the Miabot robots in the images, and publishes the poses of the robots as a PoseArray. It also subscribes to the state_estimate topic (PoseArray messages) so that the published PoseArray maintains the same order as the state_estimate. It matches the detected robots with the estimated poses using the Hungarian algorithm via the munkres library.

## @author Brendan Andrade


import roslib
roslib.load_manifest('dcsl_vision_tracker')
import rospy
from geometry_msgs.msg import PoseArray,Pose
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv
import math as m

from dcsl_vision_tracker_API import DcslBelugaTracker, DcslPose

## This class allows detection of robots in an image,  orders these readings based on state estimates, and publishes them as a PoseArray.
class BelugaTracker:
    
    ## Creates publishers and subscribers, loads background image and nRobots parameter, and creates CvBridge object.
    def __init__(self):
        # Create publisher and subscriber objects
        self.image0_pub = rospy.Publisher("/camera0/tracked_image", Image)
        self.image1_pub = rospy.Publisher("/camera1/tracked_image", Image)
        self.image2_pub = rospy.Publisher("/camera2/tracked_image", Image)
        self.image3_pub = rospy.Publisher("/camera3/tracked_image", Image)
        self.measurement_pub = rospy.Publisher("planar_measurements", PoseArray)
        self.image0_sub = rospy.Subscriber("/camera0/image_raw", Image, self.image0Callback)
        self.image1_sub = rospy.Subscriber("/camera1/image_raw", Image, self.image1Callback)
        self.image2_sub = rospy.Subscriber("/camera2/image_raw", Image, self.image2Callback)
        self.image3_sub = rospy.Subscriber("/camera3/image_raw", Image, self.image3Callback)
        self.state_sub = rospy.Subscriber("state_estimate", PoseArray, self.stateCallback)
        self.bridge = CvBridge()

        # Load background images
        location0 = rospy.get_param('/vision_tracker/background_image0')
        self.background0 = cv.LoadImageM(location0, cv.CV_LOAD_IMAGE_COLOR)
        location1 = rospy.get_param('/vision_tracker/background_image1')
        self.background1 = cv.LoadImageM(location1, cv.CV_LOAD_IMAGE_COLOR)
        location2 = rospy.get_param('/vision_tracker/background_image2')
        self.background2 = cv.LoadImageM(location2, cv.CV_LOAD_IMAGE_COLOR)
        location3 = rospy.get_param('/vision_tracker/background_image3')

        # Load masks
        location0 = rospy.get_param('/vision_tracker/mask0')
        self.mask0 = cv.LoadImageM(location0, cv.CV_LOAD_IMAGE_GREYSCALE)
        location1 = rospy.get_param('/vision_tracker/mask1')
        self.mask1 = cv.LoadImageM(location1, cv.CV_LOAD_IMAGE_GREYSCALE)
        location2 = rospy.get_param('/vision_tracker/mask2')
        self.mask2 = cv.LoadImageM(location2, cv.CV_LOAD_IMAGE_GREYSCALE)
        location3 = rospy.get_param('/vision_tracker/mask3')
        self.mask3 = cv.LoadImageM(location3, cv.CV_LOAD_IMAGE_GREYSCALE)

        # Create tracker object from API
        threshold = 100
        erode_iterations = 1
        min_blob_size = 500
        max_blob_size = 2000
        scale = 1.7526/1024.0 #meters/pixel
        image_width = self.background0.width
        image_height = self.background0.height
        self.storage = cv.CreateMemStorage()
        self.tracker = DcslBelugaTracker(background, threshold, erode_iterations, min_blob_size, max_blob_size, scale, self.storage, image_width, image_height)

        '''
        # For testing
        temp1 = DcslPose()
        temp1.set_position((0,0,0))
        temp1.set_quaternion((0,0,0,0))
        temp2 = DcslPose()
        temp2.set_position((-1,-1,0))
        temp2.set_quaternion((0,0,0,0))
        self.current_states = [temp1,temp2]
        '''

    ## Callback function for when new images are received on camera0. Senses positions of robots, sorts them into the correct order, and publishes readings and image.
    #
    # @param data is the image data received from the /camera/image_raw topic
    def image0Callback(self, data):
        camera_number = 0;
        
        # Bridge image from ROS message to OpenCV
        try:
            working_image = self.bridge.imgmsg_to_cv(data, "mono8")
        except CvBridgeError, e:
            print e
        
        # Find poses and place into a message
        measurements, image_poses, contours = self.tracker.get_poses(working_image, self.current_states, camera_number)
        pose_array = self._dcsl_pose_to_ros_pose(image_poses)

        # Publish measuremented poses
        self.measurement_pub.publish(pose_array)

        # Overlay tracking information
        output_image = self._tracking_overlay(working_image, image_poses, contours)
               
        # Publish image with overlay
        try:
            self.image0_pub.publish(self.bridge.cv_to_imgmsg(output_image,"bgr8"))
        except CvBridgeError, e:
            print e       

    ## Callback function for when new images are received on camera1. Senses positions of robots, sorts them into the correct order, and publishes readings and image.
    #
    # @param data is the image data received from the /camera/image_raw topic
    def image1Callback(self, data):
        camera_number = 1;
        
        # Bridge image from ROS message to OpenCV
        try:
            working_image = self.bridge.imgmsg_to_cv(data, "mono8")
        except CvBridgeError, e:
            print e
        
        # Find poses and place into a message
        measurements, image_poses, contours = self.tracker.get_poses(working_image, self.current_states, camera_number)
        pose_array = self._dcsl_pose_to_ros_pose(image_poses)

        # Publish measuremented poses
        self.measurement_pub.publish(pose_array)

        # Overlay tracking information
        output_image = self._tracking_overlay(working_image, image_poses, contours)
               
        # Publish image with overlay
        try:
            self.image1_pub.publish(self.bridge.cv_to_imgmsg(output_image,"bgr8"))
        except CvBridgeError, e:
            print e      

    ## Callback function for when new images are received on camera1. Senses positions of robots, sorts them into the correct order, and publishes readings and image.
    #
    # @param data is the image data received from the /camera/image_raw topic
    def image2Callback(self, data):
        camera_number = 2;
        
        # Bridge image from ROS message to OpenCV
        try:
            working_image = self.bridge.imgmsg_to_cv(data, "mono8")
        except CvBridgeError, e:
            print e
        
        # Find poses and place into a message
        measurements, image_poses, contours = self.tracker.get_poses(working_image, self.current_states, camera_number)
        pose_array = self._dcsl_pose_to_ros_pose(image_poses)

        # Publish measuremented poses
        self.measurement_pub.publish(pose_array)

        # Overlay tracking information
        output_image = self._tracking_overlay(working_image, image_poses, contours)
               
        # Publish image with overlay
        try:
            self.image2_pub.publish(self.bridge.cv_to_imgmsg(output_image,"bgr8"))
        except CvBridgeError, e:
            print e      

    ## Callback function for when new images are received on camera1. Senses positions of robots, sorts them into the correct order, and publishes readings and image.
    #
    # @param data is the image data received from the /camera/image_raw topic
    def image3Callback(self, data):
        camera_number = 3;
        
        # Bridge image from ROS message to OpenCV
        try:
            working_image = self.bridge.imgmsg_to_cv(data, "mono8")
        except CvBridgeError, e:
            print e
        
        # Find poses and place into a message
        measurements, image_poses, contours = self.tracker.get_poses(working_image, self.current_states, camera_number)
         pose_array = self._dcsl_pose_to_ros_pose(image_poses)

        # Publish measuremented poses
        self.measurement_pub.publish(pose_array)

        # Overlay tracking information
        output_image = self._tracking_overlay(working_image, image_poses, contours)
               
        # Publish image with overlay
        try:
            self.image3_pub.publish(self.bridge.cv_to_imgmsg(output_image,"bgr8"))
        except CvBridgeError, e:
            print e  

    def _dcsl_pose_to_ros_pose(self, image_poses):
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
        return pose_array

    def _tracking_overlay(self, image, image_poses, contours):

        # Create BGR8 image to be output
        output_image = cv.CreateMat(working_image.height,working_image.width,cv.CV_8UC3)
        cv.Merge(working_image,working_image,working_image, None, output_image)

        # Draw line and circle for heading and position
        length = 15.0
        radius = 5
        cyan = cv.RGB(0,255,255)
        for point in image_poses:
            end = (int(point.position_x()+m.cos(point.quaternion_z())*length),int(point.position_y()-m.sin(point.quaternion_z())*length))
            center = (int(point.position_x()), int(point.position_y()))
            cv.Line(output_image, center, end, cyan)
            cv.Circle(output_image, center, radius, cyan)
        # Draw contours
        cv.DrawContours(output_image, contours, cyan, cyan, 2)

        return output_image


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

## Runs on the startup of the node. Initializes the node and creates the BelugaTracker object.
def main():
    rospy.init_node('dcsl_beluga_tracker')
    tracker = BelugaTracker()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
        cv.DestroyAllWindows()


if __name__ == '__main__':
    main()

# http://www.ros.org/wiki/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython
