#!/usr/bin/env python
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

        output_image = cv.CreateMat(working_image.height,working_image.width,cv.CV_8UC3)
        cv.Merge(working_image,working_image,working_image, None, output_image)
        #cv.CvtColor(output_image, working_image,cv.CV_GRAY2BGR)        

        cv.AbsDiff(working_image,self.background,working_image)
        threshold = 100
        cv.Threshold(working_image,working_image,threshold,255,cv.CV_THRESH_BINARY)
        erodeIterations = 3
        cv.Erode(working_image, working_image, None, erodeIterations)
        contours = cv.FindContours(working_image, cv.CreateMemStorage(), cv.CV_RETR_CCOMP, cv.CV_CHAIN_APPROX_SIMPLE)
        if contours:
            box = cv.MinAreaRect2(contours,cv.CreateMemStorage())
            center = (int(box[0][0]),int(box[0][1]))
            red = cv.RGB(255,0,0)
            blue = cv.RGB(0,0,255)
            cv.DrawContours(output_image, contours, red, blue,2)
            radius = 5
            cv.Circle(output_image, center, radius, blue) 
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
