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
from dynamic_reconfigure.server import Server
from dcsl_vision_tracker.cfg import dcsl_beluga_tracker_configConfig as Config
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
        self.image0_sub = rospy.Subscriber("/camera0/image_rect_color", Image, self.image0Callback)
        self.image1_sub = rospy.Subscriber("/camera1/image_rect_color", Image, self.image1Callback)
        self.image2_sub = rospy.Subscriber("/camera2/image_rect_color", Image, self.image2Callback)
        self.image3_sub = rospy.Subscriber("/camera3/image_rect_color", Image, self.image3Callback)
        self.state_sub = rospy.Subscriber("state_estimate", PoseArray, self.stateCallback)
        self.bridge = CvBridge()
        self.srv = Server(Config, self.parameter_callback)

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

        # Create tracker object from API
        binary_threshold = 25
        erode_iterations = 4
        min_blob_size = 20
        max_blob_size = 2000
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
        matched_world_poses, matched_image_poses, contours = self.tracker.get_poses(working_image, self.current_states, camera_number)
        world_ros_array = self._dcsl_pose_to_ros_pose(matched_world_poses)
        image_ros_array = self._dcsl_pose_to_ros_pose(matched_image_poses)

        # Publish measuremented poses
        self.measurement_pub.publish(world_ros_array)

        # Overlay tracking information
        output_image = self._tracking_overlay(working_image, matched_image_poses, contours)
               
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
            working_image = self.bridge.imgmsg_to_cv(data, "bgr8")
        except CvBridgeError, e:
            print e
        
        # Find poses and place into a message
        matched_world_poses, matched_image_poses, contours = self.tracker.get_poses(working_image, self.current_states, camera_number)
        world_ros_array = self._dcsl_pose_to_ros_pose(matched_world_poses)
        image_ros_array = self._dcsl_pose_to_ros_pose(matched_image_poses)

        # Publish measuremented poses
        self.measurement_pub.publish(world_ros_array)

        # Overlay tracking information
        output_image = self._tracking_overlay(working_image, matched_image_poses, contours)
               
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
            working_image = self.bridge.imgmsg_to_cv(data, "bgr8")
        except CvBridgeError, e:
            print e
        
        # Find poses and place into a message
        matched_world_poses, matched_image_poses, contours = self.tracker.get_poses(working_image, self.current_states, camera_number)
        world_ros_array = self._dcsl_pose_to_ros_pose(matched_world_poses)
        image_ros_array = self._dcsl_pose_to_ros_pose(matched_image_poses)

        # Publish measuremented poses
        self.measurement_pub.publish(world_ros_array)

        # Overlay tracking information
        output_image = self._tracking_overlay(working_image, matched_image_poses, contours)
               
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
            working_image = self.bridge.imgmsg_to_cv(data, "bgr8")
        except CvBridgeError, e:
            print e
        
        # Find poses and place into a message
        matched_world_poses, matched_image_poses, contours = self.tracker.get_poses(working_image, self.current_states, camera_number)
        world_ros_array = self._dcsl_pose_to_ros_pose(matched_world_poses)
        image_ros_array = self._dcsl_pose_to_ros_pose(matched_image_poses)

        # Publish measuremented poses
        self.measurement_pub.publish(world_ros_array)

        # Overlay tracking information
        output_image = self._tracking_overlay(working_image, matched_image_poses, contours)
               
        # Publish image with overlay
        try:
            self.image3_pub.publish(self.bridge.cv_to_imgmsg(output_image,"bgr8"))
        except CvBridgeError, e:
            print e  

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
        for point in image_poses:
            if point.position[0] is not None:
                end = (int(point.position_x()+m.cos(point.quaternion_z())*length),int(point.position_y()-m.sin(point.quaternion_z())*length))
                center = (int(point.position_x()), int(point.position_y()))
                cv.Line(output_image, center, end, red)
                cv.Circle(output_image, center, radius, red)
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
            pos_z = pose.position.z
            theta = pose.orientation.z
            temp.set_position((pos_x,pos_y,pos_z))
            temp.set_quaternion((0,0,theta,0))
            self.current_states.append(temp)

    ## Callback function for parameter updates.
    #
    #@param config data received for parameter update
    #@param level
    def parameter_callback(self, config, level):
        if "tracker" in self.__dict__:
            self.tracker.threshold = config["binary_threshold"]
            self.tracker.erode_iterations = config["erode_iterations"]
            self.tracker.min_blob_size = config["min_blob_size"]
            self.tracker.max_blob_size = config["max_blob_size"]
            self.tracker.scale = config["scale"]
            self.tracker.camera_height = config["camera_height"]
        rospy.logdebug
        return config


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
