#!/usr/bin/env python

## @file
# A node that subscribes to the /camera/image_raw topic with Image messages, uses OpenCV to find the poses of the Miabot robots in the images, and publishes the poses of the robots as a PoseArray. It also subscribes to the state_estimate topic (PoseArray messages) so that the published PoseArray maintains the same order as the state_estimate. It matches the detected robots with the estimated poses using the Hungarian algorithm via the munkres library.

## @author Brendan Andrade

# Not used with catkin
# import roslib
# roslib.load_manifest('dcsl_vision_tracker')

import actionlib
import rospy
from geometry_msgs.msg import PoseArray,Pose
from dcsl_messages.msg import StateArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from dynamic_reconfigure.server import Server
from dcsl_vision_tracker.cfg import dcsl_miabot_tracker_configConfig as Config
from dcsl_vision_tracker.msg import *

import cv
import math as m



from dcsl_vision_tracker_API import DcslMiabotTracker, DcslPose

## This class allows detection of robots in an image,  orders these readings based on state estimates, and publishes them as a PoseArray.
class miabot_tracker:
    
    _feedback = ToggleTrackingFeedback()
    _result = ToggleTrackingResult()

    ## Creates publishers and subscribers, loads background image and nRobots parameter, and creates CvBridge object.
    def __init__(self):

        self.n_cameras = 4
        self.translation_vectors = [[1.5, -1, 4], [1.5, 1, 4], [-1.5, 1, 4], [-1.5, -1, 4]]

        # Get initial states
        init_poses = rospy.get_param('initial_poses', [[0.1, 0., 0., 0.],[-0.1, 0., 0., 0.],[0., 0.1, 0., 0.],[0., -0.1, 0., 0.], [0.2, 0., 0., 0.], [-0.2, 0., 0., 0.], [0., 0., 0., 0.]])
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
        self.tracker = None #tracker initialized in parameter server callback to get defaults from server.
        
        # Start dynamic reconfigure server
        self.srv = Server(Config, self.parameter_callback)
        
        # Create publisher and subscriber objects
        self.image_pub_array = []
        self.image_sub_array = []
        for i in xrange(0, self.n_cameras):
            cam_string = "/camera" + str(i)
            self.image_pub_array.append(rospy.Publisher(cam_string+"/tracked_image", Image))
            self.image_sub_array.append(rospy.Subscriber(cam_string+"/image_rect", Image, lambda data, cam_id=i: self.imageCallback(data, cam_id)))

        self.measurement_pub = rospy.Publisher("/planar_measurements", PoseArray)
        self.state_sub = rospy.Subscriber("state_estimate", StateArray, self.stateCallback)

        # Create CvBridge object for conversion from ROS message to OpenCV image
        self.bridge = CvBridge()
        
        # Create actionlib server
        self._action_name = "dcsl_vision_tracker"
        self.server = actionlib.SimpleActionServer(self._action_name, ToggleTrackingAction, self.toggle_tracking, False)
        self.server.start()

        
    ## Callback function for when new images are received. Senses positions of robots, sorts them into the correct order, publishes readings, and displays image.
    #
    # @param data is the image data received from the /camera/image_raw topic
    def imageCallback(self,data, camera_id):
        # Bridge image from ROS message to OpenCV
        try:
            working_image = self.bridge.imgmsg_to_cv(data, "mono8")
        except CvBridgeError, e:
            print e
        
        
        # Find poses and place into a message
        measurements, image_poses, contours = self.tracker.get_poses(working_image, self.current_states, camera_id)
        # Only output_measurements if output_measurements is true (set by action server)
        if self.output_measurements:
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

            # Publish measuremented poses
            pose_array.header.stamp = data.header.stamp
            self.measurement_pub.publish(pose_array)

        # Create BGR8 image to be output
        output_image = cv.CreateMat(working_image.height,working_image.width,cv.CV_8UC3)
        cv.Merge(working_image,working_image,working_image, None, output_image)

        # Overlay position, heading, and contour
        length = 15.0
        radius = 5
        text_offset = (-30, -20)
        hscale = 0.5
        vscale = 0.7
        font = cv.InitFont(cv.CV_FONT_HERSHEY_SIMPLEX, hscale, vscale)
        cyan = cv.RGB(0,255,255)
        red = cv.RGB(255, 0, 0)
        green = cv.RGB(0, 255, 0)
        for index, point in enumerate(image_poses):
            if point.detected:
                end = (int(point.position_x()+m.cos(point.quaternion_z())*length),int(point.position_y()-m.sin(point.quaternion_z())*length))
                center = (int(point.position_x()), int(point.position_y()))
                cv.Line(output_image, center, end, red)
                cv.Circle(output_image, center, radius, red)
                cv.PutText(output_image, str("Robot ")+str(index), (center[0]+text_offset[0], center[1]+text_offset[1]), font, red)
        cv.DrawContours(output_image, contours, cyan, cyan, 2)
        if self.output_measurements:
            status = "Currently outputting pose measurements"
            color = green
        else:
            status = "Not outputting pose measurements to system"
            color = red
        cv.PutText(output_image, status, (5, 25), font, color)

        # Publish image with overlay
        try:
            self.image_pub_array[camera_id].publish(self.bridge.cv_to_imgmsg(output_image,"bgr8"))
        except CvBridgeError, e:
            print e        

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
                    theta = state.pose.orientation.z
                    temp.set_position((pos_x,pos_y,0))
                    temp.set_quaternion((0,0,theta,0))
                else:
                    temp = self.initial_states[i]
                self.current_states.append(temp)

    ## Call back function for parameter updates.
    #
    # @param config data received for parameter update
    # @param level
    def parameter_callback(self, config, level):
        if self.tracker is None:

            # Load background images
            background_list = []
            for i in xrange(0, self.n_cameras):
                location = rospy.get_param('/vision_tracker/background_image' + str(i))
                background = cv.LoadImageM(location,cv.CV_LOAD_IMAGE_GRAYSCALE)
                background_list.append(background)

            threshold = int(config["binary_threshold"])
            erode_iterations = int(config["erode_iterations"])
            min_blob_size = int(config["min_blob_size"])
            max_blob_size = int(config["max_blob_size"])
            scale = config["scale"]
            image_width = 1280
            image_height = 960
            self.storage = cv.CreateMemStorage()
            self.tracker = DcslMiabotTracker(background_list, [None]*4, threshold, erode_iterations, min_blob_size, 
                                             max_blob_size, self.storage, image_width, image_height, scale, self.translation_vectors)
        else:
            self.tracker.threshold = int(config["binary_threshold"])
            self.tracker.erode_iterations = int(config["erode_iterations"])
            self.tracker.min_blob_size = int(config["min_blob_size"])
            self.tracker.max_blob_size = int(config["max_blob_size"])
            self.tracker.scale = config["scale"]
        rospy.logdebug("""Reconfigure Request: {binary_threshold}, {erode_iterations}, {min_blob_size}, {max_blob_size}, {scale}""".format(**config))
        return config

    ##
    #
    #
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
