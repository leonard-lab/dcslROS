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

    _bg_feedback = GenerateBackgroundFeedback()
    _bg_result = GenerateBackgroundResult()

    ## Creates publishers and subscribers, loads background image and nRobots parameter, and creates CvBridge object.
    def __init__(self):
        
        # Get initial states
        init_poses = rospy.get_param('initial_poses', [[-2., -0.5, 1.75, 0.], [-0.5, -2.0, 1.75, 0.], [2.0, 0.0, 1.75, 0.], [1., 0., 1.75, 0.]])
        n_robots = rospy.get_param('/n_robots')

        self.n_cameras = 4
        self.generate_bg_flag = False

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
        self.image_pub_array = []
        self.image_sub_array = []
        for i in xrange(0, self.n_cameras):
            cam_string = "/camera" + str(i)
            self.image_pub_array.append(rospy.Publisher(cam_string+"/tracked_image", Image))
            self.image_sub_array.append(rospy.Subscriber(cam_string+"/image_rect_color", Image, lambda data, cam_id=i: self.imageCallback(data, cam_id), queue_size = 1))
        self.measurement_pub = rospy.Publisher("planar_measurements", PoseArray)
        self.state_sub = rospy.Subscriber("state_estimate", StateArray, self.stateCallback)
        self.bridge = CvBridge()

        # Create action servers
        tracking_action_name = "dcsl_vt_toggle_tracking"
        self.server = actionlib.SimpleActionServer(tracking_action_name, ToggleTrackingAction, self.toggle_tracking, False)
        self.server.start()
        
        background_action_name = "dcsl_vt_background"
        self.background_server = actionlib.SimpleActionServer(background_action_name, GenerateBackgroundAction, auto_start = False)
        self.background_server.register_goal_callback(self.generate_background_cb) # This action server uses goal callback strategy whereas the other two use execute callbacks. See actionlib wiki.
        self.background_server.register_preempt_callback(self.gb_preempt_cb)
        self.background_server.start()


    ## Callback function for when new images are received on camera0. Senses positions of robots, sorts them into the correct order, and publishes readings and image.
    #
    # @param data is the image data received from the /camera/image_raw topic
    def imageCallback(self, data, camera_number):
        
        # Bridge image from ROS message to OpenCV
        try:
            working_image = self.bridge.imgmsg_to_cv(data, "bgr8")
        except CvBridgeError, e:
            print e
        
        if self.generate_bg_flag:
            n_images = 5 # Number of images to average for each camera

            # Perform average
            if self.count[camera_number] == 0:
                self.background_list[camera_number] = working_image
                self.count[camera_number] = self.count[camera_number] + 1
            elif self.count[camera_number] < n_images:
                cv.AddWeighted(self.background_list[camera_number], float(self.count[camera_number]-1.0)/float(self.count[camera_number]), 
                                                                working_image, 1.0/float(self.count[camera_number]), 0, self.background_list[camera_number])
                self.count[camera_number] += 1

            # Publish feedback
            self._bg_feedback.progress = float(sum(self.count))/float(n_images*self.n_cameras)
            self.background_server.publish_feedback(self._bg_feedback)
            
            # If reached n_images for all cameras
            if sum(self.count) == n_images*self.n_cameras:
                # Turn off background generation
                self.generate_bg_flag = False
                for i in xrange(0, self.n_cameras):
                    # Save background images
                    cv.SaveImage(self.background_locations[i], self.background_list[i])
                    # Update tracker
                    self.tracker.set_background_list(self.background_list)
                    # Action result feedback
                    self._bg_result.successful = True
                    if self.background_server.is_active():
                        self.background_server.set_succeeded(self._bg_result)
                    # Reset count
                    self.count = [0]*self.n_cameras
                    
            return # return and don't do tracking


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
            self.image_pub_array[camera_number].publish(self.bridge.cv_to_imgmsg(output_image,"bgr8"))
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
            self.background_list = background_list
            self.background_locations = [location0, location1, location2, location3]

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

    ##
    #
    #
    def generate_background_cb(self):
        goal = self.background_server.accept_new_goal()
        if goal.generate == True:
            self.background_list = [None]*self.n_cameras
            self.count = [0]*self.n_cameras
            self.generate_bg_flag = True

    ##
    #
    #
    def gb_preempt_cb(self):
        self.background_list = [None]*self.n_cameras
        self.count = [0]*self.n_cameras
        self.background_server.set_preempted()


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
