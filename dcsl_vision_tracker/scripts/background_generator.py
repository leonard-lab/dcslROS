#!/usr/bin/env python

## @file
# A node that generators a background image. It averages the first five images received on camera/image_raw and saves them to the location indicated by the /background_generator/image_name parameter.

## @author Brendan Andrade

import roslib; roslib.load_manifest('dcsl_vision_tracker')
import rospy
import cv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

## This class allows the generator of a background image for use with miabot_tracker.py
class background_generator:
    ## Creates the subscriber, sets properties defining how many images to average, and creates teh CvBridge object.
    def __init__(self):
        self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.callback)
        self.bridge = CvBridge()
        self.n = 5
        self.count = 1

    ## Callback function for the image subscriber. It adds the received image to the average image and outputs the background image once self.n images have been added.
    #
    # @param data is the iamge data from the image subscriber.
    def callback(self,data):
        # Bridge the image from ROS to OpenCV
        try:
            cv_image = self.bridge.imgmsg_to_cv(data, "mono8")
        except CvBridgeError, e:
            print e
            
        # Create the background image object if this is the first trigger of the callback
        if self.count == 1:
            print "Initializing image"
            self.background_image = cv.CreateMat(cv_image.rows, cv_image.cols, cv.CV_64FC1)
            cv.SetZero(self.background_image)
        
        # Add image to total image (Pixel by pixel addition of grayscale values
        cv.Acc(cv_image,self.background_image)
        
        # If self.n images have been added, take average and output
        if self.count == self.n:
            # For each pixel divide by self.n to find average image
            cv.ConvertScale(self.background_image,self.background_image,1.0/float(self.n));
            # Get image name and output image
            image_name = rospy.get_param('/background_generator/image_name')
            cv.SaveImage(image_name,self.background_image)  
            print "Background image generated...node shutting down"
            # Shutdown node
            rospy.signal_shutdown("Background image generated...node shutting down")

        self.count += 1
        #Display image in window
        cv.ShowImage("Image Window", self.background_image)
        cv.WaitKey(3)

## Main function triggered when node begins.
#
# Creates the node object and background_generator object.        
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
