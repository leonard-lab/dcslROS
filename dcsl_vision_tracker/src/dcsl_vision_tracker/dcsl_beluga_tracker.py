#!/usr/bin/env python

## @file
# A node that subscribes to the /cameraX/image_raw topics (where X is the number of the camera) with Image messages, uses OpenCV to find the poses of the Miabot robots in the images, and publishes the poses of the robots as a PoseArray. It also subscribes to the state_estimate topic (PoseArray messages) so that the published PoseArray maintains the same order as the state_estimate. It matches the detected robots with the estimated poses using the Hungarian algorithm via the munkres library.

## @author Brendan Andrade

# Not used with catkin
# import roslib
# roslib.load_manifest('dcsl_vision_tracker')

import actionlib
import rospy
from geometry_msgs.msg import PoseArray,Pose
from sensor_msgs.msg import Image
from dcsl_messages.msg import StateArray
from cv_bridge import CvBridge, CvBridgeError
from dynamic_reconfigure.server import Server
from dcsl_vision_tracker.cfg import dcsl_beluga_tracker_configConfig as Config
from dcsl_vision_tracker.msg import *

import cv
import math as m

from dcsl_vision_tracker_API import DcslBelugaTracker, DcslPose

## This class allows detection of robots in an image,  orders these readings based on state estimates, and publishes them as a PoseArray.
class BelugaTracker:
    
    _feedback = ToggleTrackingFeedback()
    _result = ToggleTrackingResult()

    ## Creates publishers and subscribers, loads background image and nRobots parameter, and creates CvBridge object.
    def __init__(self):
        
        # Get initial states
        init_poses = rospy.get_param('initial_poses', [[-2., -0.5, 1.75, 0.], [-0.5, -2.0, 1.75, 0.], [2.0, 0.0, 1.75, 0.], [1., 0., 1.75, 0.]])
        n_robots = rospy.get_param('/n_robots')

        self.initial_states = []
        for i, pose in enumerate(init_poses):
            if i < n_robots:
                temp = DcslPose()
                temp.set_position((pose[0], pose[1], pose[2]))
                temp.set_quaternion((0., 0., pose[3], 0.))
                self.initial_states.append(temp)
        self.current_states = self.initial_states
        self.output_measurements = False
        self.receive_states = True

        self.tracker = None # Tracker initialized in parameter server callback to get defaults from server.
    
        self.srv = Server(Config, self.parameter_callback) # Dynamic reconfigure server

        
        # Create publisher and subscriber objects
        self.image0_pub = rospy.Publisher("/camera0/tracked_image", Image)
        self.image1_pub = rospy.Publisher("/camera1/tracked_image", Image)
        self.image2_pub = rospy.Publisher("/camera2/tracked_image", Image)
        self.image3_pub = rospy.Publisher("/camera3/tracked_image", Image)
        self.measurement_pub = rospy.Publisher("planar_measurements", PoseArray)
        self.image0_sub = rospy.Subscriber("/camera0/image_rect_color", Image, self.image0Callback)
        self.image1_sub = rospy.Subscriber("/camera1/image_rect_color", Image, self.image1Callback)
        self.image2_sub = rospy.Subscriber("/camera2/image_rect_color", Image, self.image2Callback)
        self.image3_sub = rospy.Subscriber("/camera3/image_rect_color", Image, self.image3Callback)
        self.state_sub = rospy.Subscriber("state_estimate", StateArray, self.stateCallback)
        self.bridge = CvBridge()

        # Create action server
        self._action_name = "dcsl_vision_tracker"
        self.server = actionlib.SimpleActionServer(self._action_name, ToggleTrackingAction, self.toggle_tracking, False)
        self.server.start()
        
        '''
        # Create tracker object from API
        binary_threshold = 5
        erode_iterations = 4
        min_blob_size = 350
        max_blob_size = 1750
        scale = pow(1.45/3.05*1.0/204.0, -1) # 1/pixels
        camera_height = 3.14 # meters
        refraction_ratio = 1.0/1.333 #refractive index of air/refractive index of water
        image_width = background_list[0].width
        image_height = background_list[0].height
        R_cam1 = (0.984, 1.956, 5.52)
        R_cam2 = (-1.021, 1.956, 5.52)
        R_cam3 = (-1.057, -2.005, 5.52)
        R_cam4 = (0.996, -1.908, 5.52)
        translation_offset_list = [R_cam4, R_cam1, R_cam2, R_cam3]
        self.storage = cv.CreateMemStorage()
        self.tracker = DcslBelugaTracker(background_list, mask_list, binary_threshold, erode_iterations, min_blob_size, max_blob_size, self.storage, image_width, image_height, scale, translation_offset_list, camera_height, refraction_ratio)
        
        
        # For testing
        temp1 = DcslPose()
        temp1.set_position((-2,-2,0))
        temp1.set_quaternion((0,0,0,0))
        self.current_states = [temp1]
        '''

    ## Callback function for when new images are received on camera0. Senses positions of robots, sorts them into the correct order, and publishes readings and image.
    #
    # @param data is the image data received from the /camera/image_raw topic
    def image0Callback(self, data):
        camera_number = 0;
        
        # Bridge image from ROS message to OpenCV
        try:
            working_image = self.bridge.imgmsg_to_cv(data, "bgr8")
        except CvBridgeError, e:
            print e
        
        # Find poses and place into a message
        storage = cv.CreateMemStorage()
        matched_world_poses, matched_image_poses, contours = self.tracker.get_poses(working_image, self.current_states, camera_number, storage)
        world_ros_array = self._dcsl_pose_to_ros_pose(matched_world_poses)
        image_ros_array = self._dcsl_pose_to_ros_pose(matched_image_poses)

        # Publish measuremented poses
        if self.output_measurements:
            world_ros_array.header.stamp = data.header.stamp
            self.measurement_pub.publish(world_ros_array)

        # Overlay tracking information
        output_image = self._tracking_overlay(working_image, matched_image_poses, contours)
               
        # Publish image with overlay
        try:
            self.image0_pub.publish(self.bridge.cv_to_imgmsg(output_image,"bgr8"))
        except CvBridgeError, e:
            print e       

        del contours, storage

    ## Callback function for when new images are received on camera1. Senses positions of robots, sorts them into the correct order, and publishes readings and image.
    #
    # @param data is the image data received from the /camera/image_raw topic
    def image1Callback(self, data):
        camera_number = 1;
        
        # Bridge image from ROS message to OpenCV
        try:
            working_image = self.bridge.imgmsg_to_cv(data, "bgr8")
        except CvBridgeError, e:
            print e
        
        # Find poses and place into a message
        storage = cv.CreateMemStorage()
        matched_world_poses, matched_image_poses, contours = self.tracker.get_poses(working_image, self.current_states, camera_number, storage)
        world_ros_array = self._dcsl_pose_to_ros_pose(matched_world_poses)
        image_ros_array = self._dcsl_pose_to_ros_pose(matched_image_poses)

        # Publish measuremented poses
        if self.output_measurements:
            world_ros_array.header.stamp = data.header.stamp
            self.measurement_pub.publish(world_ros_array)

        # Overlay tracking information
        output_image = self._tracking_overlay(working_image, matched_image_poses, contours)
               
        # Publish image with overlay
        try:
            self.image1_pub.publish(self.bridge.cv_to_imgmsg(output_image,"bgr8"))
        except CvBridgeError, e:
            print e      

        del contours, storage

    ## Callback function for when new images are received on camera1. Senses positions of robots, sorts them into the correct order, and publishes readings and image.
    #
    # @param data is the image data received from the /camera/image_raw topic
    def image2Callback(self, data):
        camera_number = 2;
        
        # Bridge image from ROS message to OpenCV
        try:
            working_image = self.bridge.imgmsg_to_cv(data, "bgr8")
        except CvBridgeError, e:
            print e
        
        # Find poses and place into a message
        storage = cv.CreateMemStorage()
        matched_world_poses, matched_image_poses, contours = self.tracker.get_poses(working_image, self.current_states, camera_number, storage)
        world_ros_array = self._dcsl_pose_to_ros_pose(matched_world_poses)
        image_ros_array = self._dcsl_pose_to_ros_pose(matched_image_poses)

        # Publish measuremented poses
        if self.output_measurements:
            world_ros_array.header.stamp = data.header.stamp
            self.measurement_pub.publish(world_ros_array)

        # Overlay tracking information
        output_image = self._tracking_overlay(working_image, matched_image_poses, contours)
               
        # Publish image with overlay
        try:
            self.image2_pub.publish(self.bridge.cv_to_imgmsg(output_image,"bgr8"))
        except CvBridgeError, e:
            print e      

        del contours, storage

    ## Callback function for when new images are received on camera1. Senses positions of robots, sorts them into the correct order, and publishes readings and image.
    #
    # @param data is the image data received from the /camera/image_raw topic
    def image3Callback(self, data):
        camera_number = 3;
        
        # Bridge image from ROS message to OpenCV
        try:
            working_image = self.bridge.imgmsg_to_cv(data, "bgr8")
        except CvBridgeError, e:
            print e
        
        # Find poses and place into a message
        storage = cv.CreateMemStorage()
        matched_world_poses, matched_image_poses, contours = self.tracker.get_poses(working_image, self.current_states, camera_number, storage)
        world_ros_array = self._dcsl_pose_to_ros_pose(matched_world_poses)
        image_ros_array = self._dcsl_pose_to_ros_pose(matched_image_poses)

        # Publish measuremented poses
        if self.output_measurements:
            world_ros_array.header.stamp = data.header.stamp
            self.measurement_pub.publish(world_ros_array)

        # Overlay tracking information
        output_image = self._tracking_overlay(working_image, matched_image_poses, contours)
               
        # Publish image with overlay
        try:
            self.image3_pub.publish(self.bridge.cv_to_imgmsg(output_image,"bgr8"))
        except CvBridgeError, e:
            print e 
        
        del contours, storage

    def _dcsl_pose_to_ros_pose(self, dcsl_poses):
        pose_array = PoseArray()
        for dcsl_pose in dcsl_poses:
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
        output_image = cv.CloneMat(image)

        # Draw line and circle for heading and position
        length = 15.0
        radius = 5
        cyan = cv.RGB(0,255,255)
        red = cv.RGB(255, 0, 0)
        green = cv.RGB(0, 255, 0)
        hscale = 0.5
        vscale = 0.7
        font = cv.InitFont(cv.CV_FONT_HERSHEY_SIMPLEX, hscale, vscale)
        text_offset = (-30, -20)
        for index, point in enumerate(image_poses):
            if point.position[0] is not None:
                end = (int(point.position_x()+m.cos(point.quaternion_z())*length),int(point.position_y()-m.sin(point.quaternion_z())*length))
                center = (int(point.position_x()), int(point.position_y()))
                cv.Line(output_image, center, end, red)
                cv.Circle(output_image, center, radius, red)
                cv.PutText(output_image, str("Robot ") + str(index), (center[0] + text_offset[0], center[1] + text_offset[1]), font, red)
        # Draw contours
        cv.DrawContours(output_image, contours, cyan, cyan, 2)
        if self.output_measurements:
            status = "Outputting measurements"
            color = green
        else:
            status = "Output off"
            color = red

        return output_image


    ## Callback function for state estimates. Stores state estimates as self.currentStates.
    #
    # @param data is the message received from the subscriber.
    def stateCallback(self, data):
        if self.receive_states:
            self.current_states = []
            for i, state in enumerate(data.states):
                temp = DcslPose()
                if state.pose.orientation.w == 1:
                    pos_x = state.pose.position.x
                    pos_y = state.pose.position.y
                    pos_z = state.pose.position.z
                    theta = state.pose.orientation.z
                    temp.set_position((pos_x,pos_y,pos_z))
                    temp.set_quaternion((0,0,theta,0))
                else:
                    temp = self.initial_states[i]
                self.current_states.append(temp)

    ## Callback function for parameter updates.
    #
    #@param config data received for parameter update
    #@param level
    def parameter_callback(self, config, level):
        if self.tracker is None:

            # Load background images
            background_list = []
            location0 = rospy.get_param('/vision_tracker/background_image0')
            background_list.append(cv.LoadImageM(location0, cv.CV_LOAD_IMAGE_COLOR))
            location1 = rospy.get_param('/vision_tracker/background_image1')
            background_list.append(cv.LoadImageM(location1, cv.CV_LOAD_IMAGE_COLOR))
            location2 = rospy.get_param('/vision_tracker/background_image2')
            background_list.append(cv.LoadImageM(location2, cv.CV_LOAD_IMAGE_COLOR))
            location3 = rospy.get_param('/vision_tracker/background_image3')
            background_list.append(cv.LoadImageM(location3, cv.CV_LOAD_IMAGE_COLOR))

            # Load masks
            mask_list = []
            location0 = rospy.get_param('/vision_tracker/mask0')
            mask_list.append(cv.LoadImageM(location0, cv.CV_LOAD_IMAGE_GRAYSCALE))
            location1 = rospy.get_param('/vision_tracker/mask1')
            mask_list.append(cv.LoadImageM(location1, cv.CV_LOAD_IMAGE_GRAYSCALE))
            location2 = rospy.get_param('/vision_tracker/mask2')
            mask_list.append(cv.LoadImageM(location2, cv.CV_LOAD_IMAGE_GRAYSCALE))
            location3 = rospy.get_param('/vision_tracker/mask3')
            mask_list.append(cv.LoadImageM(location3, cv.CV_LOAD_IMAGE_GRAYSCALE))

            threshold = int(config["binary_threshold"])
            erode_iterations = int(config["erode_iterations"])
            min_blob_size = int(config["min_blob_size"])
            max_blob_size = int(config["max_blob_size"])
            scale = pow(1.45/3.05*1.0/204.0, -1) # 1/pixels
            camera_height = 3.14 # meters from water surface
            refraction_ratio = 1.0/1.333 #refractive index of air/refractive index of water
            image_width = background_list[0].width
            image_height = background_list[0].height
            R_cam1 = (0.984, 1.956, 5.52)
            R_cam2 = (-1.021, 1.956, 5.52)
            R_cam3 = (-1.057, -2.005, 5.52)
            R_cam4 = (0.996, -1.908, 5.52)
            translation_offset_list = [R_cam4, R_cam1, R_cam2, R_cam3]
            self.tracker = DcslBelugaTracker(background_list, mask_list, threshold, erode_iterations, min_blob_size, max_blob_size, image_width, image_height, scale, translation_offset_list, camera_height, refraction_ratio)
        else:
            self.tracker.threshold = int(config["binary_threshold"])
            self.tracker.erode_iterations = int(config["erode_iterations"])
            self.tracker.min_blob_size = int(config["min_blob_size"])
            self.tracker.max_blob_size = int(config["max_blob_size"])
            self.tracker.scale = int(config["scale"])
            self.tracker.camera_height = int(config["camera_height"])
        rospy.logdebug("""Reconfigure Request: {binary_threshold}, {erode_iterations}, {min_blob_size}, {max_blob_size}""".format(**config))
        return config

    def toggle_tracking(self, goal):
        # Publish feedback
        self._feedback.executing = True
        self.server.publish_feedback(self._feedback)
        
        if goal.reset == True:
            self.receive_states = False
            self.current_states = self.initial_states   
        else:
            self.receive_states = True
        
        # Turn tracking on or off
        self.output_measurements = goal.track
        
        # Send result
        self._result.tracking = goal.track
        self._feedback.executing = False
        self.server.publish_feedback(self._feedback)
        self.server.set_succeeded(self._result)


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
