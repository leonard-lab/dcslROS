#!/usr/bin/env python
import roslib; roslib.load_manifest('dcsl_vision_tracker')
import rospy
import cv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class background_generator:

    def __init__(self):
        self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.callback)
        self.bridge = CvBridge()
        self.n = 5
        self.count = 1

    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv(data, "mono8")
        except CvBridgeError, e:
            print e
            
        if self.count == 1:
            print "Initializing image"
            self.background_image = cv.CreateMat(cv_image.rows, cv_image.cols, cv.CV_64FC1)
            cv.SetZero(self.background_image)

        cv.Acc(cv_image,self.background_image)
        
        if self.count == self.n:
            
            cv.ConvertScale(self.background_image,self.background_image,1.0/float(self.n));
            image_name = rospy.get_param('/background_generator/image_name')
            cv.SaveImage(image_name,self.background_image)  
            print "Background image generated...node shutting down"
            rospy.signal_shutdown("Background image generated...node shutting down")

        self.count += 1

        cv.ShowImage("Image Window", self.background_image)
        cv.WaitKey(3)

        
def main():
    rospy.init_node('background_generator')
    bg = background_generator()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
    cv.DestroyAllWindows()

if __name__ == '__main__':
    print "Starting background generator"
    main()
