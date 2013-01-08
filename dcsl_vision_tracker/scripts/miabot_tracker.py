#!/usr/bin/env python
import math as m

import roslib
roslib.load_manifest('dcsl_vision_tracker')
import rospy
import cv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge,CvBridgeError

class miabot_tracker:

    def __init__(self):
        self.image_pub = rospy.Publisher("tracked_image",Image)
        self.background = cv.LoadImageM('background.png',cv.CV_LOAD_IMAGE_GRAYSCALE)
        self.image_sub = rospy.Subscriber("/camera/image_raw",Image,self.callback)
        self.bridge = CvBridge()

    def callback(self,data):
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
        erodeIterations = 0
        cv.Erode(working_image, working_image, None, erodeIterations)
        contours = cv.FindContours(working_image, cv.CreateMemStorage(), cv.CV_RETR_LIST, cv.CV_CHAIN_APPROX_SIMPLE, (0,0))

        red = cv.RGB(255,0,0)
        blue = cv.RGB(0,0,255)

        if contours:
            _c = contours
            #Cycle through all contours
            i = 0
            while _c is not None:
                #Find position in image
                box = cv.MinAreaRect2(_c,cv.CreateMemStorage())
                center = (int(box[0][0]),int(box[0][1]))
                centerFloat = (box[0][0],box[0][1])
                radius = 5 

                #Find orientation in image
                moments = cv.Moments(_c, False)
                mu00 = cv.GetCentralMoment(moments,0,0)
                minBlobSize = 500
                if mu00 > minBlobSize:
                    '''
                    mu11Prime = cv.GetCentralMoment(moments,1,1)/mu00
                    mu20Prime = cv.GetCentralMoment(moments,2,0)/mu00
                    mu02Prime = cv.GetCentralMoment(moments,0,2)/mu00
                    theta = 0.5*m.atan(2.0*mu11Prime/(mu20Prime-mu02Prime)) #angle of major axis of image intensity to the x axis
                    '''
                    centroidFloat = (cv.GetSpatialMoment(moments,1,0)/cv.GetSpatialMoment(moments,0,0),cv.GetSpatialMoment(moments,0,1)/cv.GetSpatialMoment(moments,0,0))
                    centroid = (int(centroidFloat[0]),int(centroidFloat[1]))
                    '''
                    if centroid[0] > center[0]:
                        theta += m.pi
                    '''
                    theta = m.atan2(centerFloat[1]-centroidFloat[1],centerFloat[0]-centroidFloat[0])
                    print theta*180.0/m.pi

                    length = 15.0
                    end = (int(center[0]+m.cos(theta)*length),int(center[1]+m.sin(theta)*length))
                    cv.Line(output_image,center,end,blue)                
                    cv.Circle(output_image, center, radius, blue)
                    cv.DrawContours(output_image, _c, red, blue, 2)

                _c = _c.h_next()
                i += 1

            #cv.DrawContours(output_image, contours, red, blue,2)

        cv.ShowImage("Image Window", output_image)
        cv.WaitKey(3)

        try:
            self.image_pub.publish(self.bridge.cv_to_imgmsg(output_image,"bgr8"))
        except CvBridgeError, e:
            print e
        
        

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
