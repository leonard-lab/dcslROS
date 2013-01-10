#!/usr/bin/env python
import math as m

import roslib
roslib.load_manifest('dcsl_vision_tracker')
import rospy
from geometry_msgs.msg import PoseArray,Pose
import cv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge,CvBridgeError


class miabot_tracker:

    def __init__(self):
        self.image_pub = rospy.Publisher("tracked_image",Image)
        self.measurement_pub = rospy.Publisher("planar_measurements", PoseArray)
        location = rospy.get_param('/vision_tracker/background_image')
        self.background = cv.LoadImageM(location,cv.CV_LOAD_IMAGE_GRAYSCALE)
        self.image_sub = rospy.Subscriber("/camera/image_raw",Image,self.imageCallback)
        self.state_sub = rospy.Subscriber("state_estimate", PoseArray, self.stateCallback)
        self.bridge = CvBridge()
        self.nRobots = rospy.get_param("/n_robots",1)

    def imageCallback(self,data):
        try:
            working_image = self.bridge.imgmsg_to_cv(data, "mono8")
        except CvBridgeError, e:
            print e

        #Create BGR8 image to be output
        output_image = cv.CreateMat(working_image.height,working_image.width,cv.CV_8UC3)
        cv.Merge(working_image,working_image,working_image, None, output_image)        

        cv.AbsDiff(working_image,self.background,working_image) #Background subtractionstorage
        threshold = 100
        cv.Threshold(working_image,working_image,threshold,255,cv.CV_THRESH_BINARY) #Threshold to create binary image
        erodeIterations = 1
        cv.Erode(working_image, working_image, None, erodeIterations)
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
                    
                    #Theta is found by drawing line from centroid of blob through center of the robot
                    theta = -m.atan2(centerFloat[1]-centroidFloat[1],centerFloat[0]-centroidFloat[0])
                    
                    #print "Theta: " + str(theta*180.0/m.pi)
                    #print "Box: " + str(box[2])
                    
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

            cv.DrawContours(output_image, contours, red, blue,2)

        cv.ShowImage("Image Window", output_image)
        cv.WaitKey(3)

        try:
            self.image_pub.publish(self.bridge.cv_to_imgmsg(output_image,"bgr8"))
        except CvBridgeError, e:
            print e

        self.matchRobots(readings)

        self.measurement_pub.publish(self.measurements)

    def stateCallback(self, data):
        self.currentStates = data       

    def matchRobots(self, readings):
        scores = []
        idList = []
        for reading in readings:
            temp = []
            for state in self.currentStates.poses:
                difference = pow(reading[0]-state.position.x,2)+pow(reading[1]-state.position.y,2)
                temp.append(difference)                
            scores.append(temp)
            idList.append(temp.index(min(temp)))
        
        '''
        for i in xrange(0,self.nRobots):
            if idList.count(i) != 1:
            '''
        for i in xrange(0,self.nRobots):
            for ID in idList:
                if ID == i:
                    p = Pose()
                    p.position.x = readings[i][0]
                    p.position.y = readings[i][1]
                    p.orientation.z = readings[i][2]
                    p.orientation.w = 1
                    self.measurements.poses.append(p)




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

## http://www.ros.org/wiki/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython
