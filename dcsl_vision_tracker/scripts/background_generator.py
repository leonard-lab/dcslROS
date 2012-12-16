#!/usr/bin/env python
import roslib; roslib.load_manifest('dcsl_vision_tracker')
import rospy
import cv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class background_generator:

    def __init__(self):
        self.image_sub = rospy.Subscriber("image_feed", Image)
        self.bridge = CvBridge()
        self.n = 5
        self.count = 1

    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv(data, "mono8")
        except CvBridgeError, e:
            print e
        if count == 1:
            self.background_image = cv.CloneImage(cv_image)
        else:
            cv.Add(cv_image,self.background_image,self.background_image)
        if count == n:
            background_image = background_image/n;
            cv.SaveImage("background.png",background_image)
        ++self.count

def main():
    rospy.init_node('background_generator')
    bg = background_generator()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
    
if __name__ == '__main__':
    main()
