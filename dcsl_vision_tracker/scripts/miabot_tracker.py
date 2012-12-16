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
        self.background = cv.LoadImageM('background.jpg',cv.CV_LOAD_IMAGE_GRAYSCALE)
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback)
        self.bridge = CvBridge()

    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv(data, "mono8")
        except CvBridgeError, e:
            print e

        ## cv.AbsDiff(cv_image,self.background,cv_image)
        cv.ShowImage("Image Window", cv_image)
        cv.WaitKey(3)
        
        try:
            self.image_pub.publish(self.bridge.cv_to_imgmsg(cv_image,"mono8"))
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
