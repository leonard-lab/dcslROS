#!/usr/bin/env python

## @file
# A node that subscribes to the cmd_vel topic with Twist messages. It converts the velocity and angular velocity in the message to wheel speed commands and sends them to the robot over a serial connection.

## @author Brendan Andrade

import sys

# Not used with catkin
# import roslib; roslib.load_manifest('dcsl_miabot_driver')

import rospy
from geometry_msgs.msg import Twist

from dcsl_miabot_API import *


##
#
#
class MiabotNode(object):
    ##
    #
    #
    def __init__(self, port):
        self.sub = rospy.Subscriber("cmd_vel", Twist, self.callback)
        self.miabot = Miabot()
	self.miabot.connect(port)
        

    ## The callback function for when a Twist message is published on the cmd_vel topic.
    #
    # This function pulls the commands from the Twist message, converts them into wheel speeds for the robot, and then writes the wheel speed command to the serial port.
    # @param cmd_vel is the message object received by the subscriber.
    def callback(self, cmd_vel):
        self.miabot.sendMotorCommand(cmd_vel.linear.x, cmd_vel.angular.z)
        
    ##
    #
    #
    def shutdown(self):
        self.miabot.stop()
        self.miabot.close()
    
## Main function which is called when the node begins
#
# Initializes the node and creates miabot_node object
def main(port):
    rospy.init_node('dcsl_miabot',anonymous=True)
    miabot_node = MiabotNode(port)
    rospy.spin()
    rospy.on_shutdown(miabot_node.shutdown())


if __name__ == "__main__":
    args = rospy.myargv(argv=sys.argv)
    port = int(args[1])
    main(port)
        
