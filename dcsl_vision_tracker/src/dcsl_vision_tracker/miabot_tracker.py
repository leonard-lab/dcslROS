#!/usr/bin/env python

## @file
# A node that subscribes to the /camera/image_raw topic with Image messages, uses OpenCV to find the poses of the robots in the images, and publishes the poses of the robots as a PoseArray. It also subscribes to the state_estimate topic (PoseArray messages) so that the published PoseArray maintains the same order as the state_estimate. It matches the detected robots with the estimated poses using the Hungarian algorithm via the munkres library.
# This node will soon be depricated in favor of a node based on the dcsl_vision_tracker_API which is under development.

## @author Brendan Andrade

import math as m

from munkres import Munkres

import roslib
roslib.load_manifest('dcsl_vision_tracker')
import rospy
from geometry_msgs.msg import PoseArray,Pose
import cv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge,CvBridgeError

## This class allows detection of robots in an image,  orders these readings based on state estimates, and publishes them as a PoseArray.
class miabot_tracker:
    
    ## Creates publishers and subscribers, loads background image and nRobots parameter, and creates CvBridge object.
    def __init__(self):
        self.image_pub = rospy.Publisher("tracked_image",Image)
        self.measurement_pub = rospy.Publisher("planar_measurements", PoseArray)
        location = rospy.get_param('/vision_tracker/background_image')
        self.background = cv.LoadImageM(location,cv.CV_LOAD_IMAGE_GRAYSCALE)
        self.image_sub = rospy.Subscriber("/camera/image_raw",Image,self.imageCallback)
        self.state_sub = rospy.Subscriber("state_estimate", PoseArray, self.stateCallback)
        self.bridge = CvBridge()
        self.nRobots = rospy.get_param("/n_robots",1)

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
        
        # Background subtraction to leave robots only in image
        cv.AbsDiff(working_image,self.background,working_image)
        # Threshold to convert to binary image
        threshold = 100
        cv.Threshold(working_image,working_image,threshold,255,cv.CV_THRESH_BINARY) #Threshold to create binary image
        # Erode image to remove small holes in binary image
        erodeIterations = 1
        cv.Erode(working_image, working_image, None, erodeIterations)
        # Create contour objects mapping the boundaries between white and black in the binary image
        contours = cv.FindContours(working_image, cv.CreateMemStorage(), cv.CV_RETR_LIST, cv.CV_CHAIN_APPROX_SIMPLE, (0,0))

        red = cv.RGB(255,0,0)
        blue = cv.RGB(0,0,255)

        self.measurements = PoseArray()
        readings = []

        if contours:
            _c = contours
            #Cycle through all contours
            while _c is not None:
                #Find position in image
                box = cv.MinAreaRect2(_c,cv.CreateMemStorage())
                center = (int(box[0][0]),int(box[0][1]))
                centerFloat = (box[0][0],box[0][1])
                radius = 5 

                #Find orientation in image
                moments = cv.Moments(_c,False)
                blobSize = cv.ContourArea(_c)
                minBlobSize = 500
                if blobSize > minBlobSize: #Ignore small blobs
                    #Find centroid of blob
                    centroidFloat = (cv.GetSpatialMoment(moments,1,0)/cv.GetSpatialMoment(moments,0,0),cv.GetSpatialMoment(moments,0,1)/cv.GetSpatialMoment(moments,0,0))
                    centroid = (int(centroidFloat[0]),int(centroidFloat[1]))
                    
                    #An estimate of theta is found by drawing line from centroid of blob through center of the robot
                    thetaEstimate = -m.atan2(centerFloat[1]-centroidFloat[1],centerFloat[0]-centroidFloat[0])
       
                    #Theta is found using the minimum area box used to find the position above
                    thetaBox = -m.pi/180.0*box[2] #Angle of the minimum area box with the x-axis in the first quadrant
                    previousError = -1.0
                    theta = None
                    stepList = [-m.pi,-m.pi*0.5,0,m.pi*0.5]
                    for step in stepList:
                        thetaTest = thetaBox+step
                        error = pow(thetaEstimate-thetaTest,2)
                        if previousError < 0:
                            theta = thetaTest
                        elif error < previousError:
                            theta = thetaTest
                        previousError = error
                    
                    #Point center and orientation on output_image
                    length = 15.0
                    end = (int(center[0]+m.cos(theta)*length),int(center[1]-m.sin(theta)*length))
                    cv.Line(output_image,center,end,blue)                
                    cv.Circle(output_image, center, radius, blue)

                    #Write sensed pose to message
                    scale = 1.7526/1024.0 #meters/pixel
                    x = (center[0]-0.5*float(working_image.width))*scale
                    y = -(center[1]-0.5*float(working_image.height))*scale
                    theta = theta
                    readings.append((x,y,theta))
                    
                #Cycle to next contour
                _c = _c.h_next() 
            #Draw contours on output image
            cv.DrawContours(output_image, contours, red, blue,2)
        #Display image
        cv.ShowImage("Image Window", output_image)
        cv.WaitKey(3)

        try:
            self.image_pub.publish(self.bridge.cv_to_imgmsg(output_image,"bgr8"))
        except CvBridgeError, e:
            print e

        self.matchRobots(readings)

        self.measurement_pub.publish(self.measurements)

    ## Callback function for state estimates. Stores state estimates as self.currentStates.
    #
    # @param data is the message received from the subscriber.
    def stateCallback(self, data):
        self.currentStates = data       

    ## Matchings readings with the robots in the current state estimate and stores the sorted measurements in a PoseArray called self.measurements.
    # 
    # To match the readings with the robots. A cost matrix is construsted of the square of the distance between each state estimate and each reading found in the image. The Hungarian algorithm is used to find the lowest cost set of matches between the readings and state_estimates. It also rejects false positive readings if they are far from the estimated positions of the robots and can handle the case when fewer robots are detected in the image than are known to exist in the system.
    # @param readings is an unsorted list of tuples represented detected poses of robots.
    def matchRobots(self, readings):
        # Create least squares cost matrix.
        costMatrix = []
        for reading in readings:
            temp = []
            for state in self.currentStates.poses:
                difference = pow(reading[0]-state.position.x,2)+pow(reading[1]-state.position.y,2)
                temp.append(difference)                
            costMatrix.append(temp)
        # Apply Hungarian algorithm to cost matrix if cost matrix exists, i.e. if robots are detected in the image.
        if len(costMatrix) != 0:
            mun = Munkres()
            # indexes is a list of the matched readings and estimates. Each entry is a tuple (X,Y) where X is the index of the reading and Y is the index of the state_estimate
            indexes = mun.compute(costMatrix) 
            # Sort readings into measurements PoseArray in correct order
            for i in xrange(0,self.nRobots):
                assigned = False
                for pair in indexes:
                    if pair[1] == i: 
                        readingsIndex = pair[0]
                        p = Pose()
                        p.position.x = readings[readingsIndex][0]
                        p.position.y = readings[readingsIndex][1]
                        p.orientation.z = readings[readingsIndex][2]
                        p.orientation.w = 1
                        self.measurements.poses.append(p)
                        assigned = True
                # Case for undetected robot
                if not assigned:
                    p = Pose()
                    p.orientation.w = 0 # Use the orientation.w entry as a binary signifier whether or not that robot has been detected in the image
                    self.measurements.poses.append(p)
        # If not robots detected return all orientation.w as zero
        else:
            for i in xrange(0,self.nRobots):
                p = Pose()
                p.orientation.w = 0
                self.measurements.poses.append(p)

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
