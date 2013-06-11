#!/usr/bin/env python

## @file
# A node that subscribes to the cmd_vel topic with Twist messages. It converts the velocity and angular velocity in the message to wheel speed commands and sends them to the robot over a serial connection.

## @author Brendan Andrade

import sys

# Not used with catkin
# import roslib; roslib.load_manifest('dcsl_miabot_driver')

import actionlib
import rospy
from geometry_msgs.msg import Twist
from dcsl_miabot_driver.msg import *

from dcsl_miabot_API import *


##
#
#
class MiabotNode(object):
    ##
    #
    #
    def __init__(self, port, bdaddr = None):
        self.port = port
        self.server = actionlib.SimpleActionServer('connect_miabot', ConnectMiabot, self.connect, False)
        self.server.start()
        self.connected = False
        self.bdaddr = bdaddr
        self.sub = rospy.Subscriber("cmd_vel", Twist, self.callback)
        

    ## The callback function for when a Twist message is published on the cmd_vel topic.
    #
    # This function pulls the commands from the Twist message, converts them into wheel speeds for the robot, and then writes the wheel speed command to the serial port.
    # @param cmd_vel is the message object received by the subscriber.
    def callback(self, cmd_vel):
        if self.connected:
            self.miabot.sendMotorCommand(cmd_vel.linear.x, cmd_vel.angular.z)
        else:
            msg = "Command received but miabot not connected"
            rospy.logwarn(msg)
        
    ##
    #
    #
    def shutdown(self):
        self.miabot.stop()
        self.miabot.close()

    ## 
    #
    #
    def connect(self, goal):
        if self.bdaddr is None:
            self.miabot = Miabot()
        else:
            self.miabot = Miabot(bdaddr)
	self.miabot.connect(self.port)
        self.connected = True
        self.server.set_succeeded()
        
## Main function which is called when the node begins
#
# Initializes the node and creates miabot_node object
def main(port = 1, bdaddr = None):
    rospy.init_node('dcsl_miabot',anonymous=True)
    if bdaddr is None:
        miabot_node = MiabotNode(port)
    else:
        miabot_node = MiabotNode(port, bdaddr = bdaddr)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down miabot...")
        miabot_node.shutdown()


if __name__ == "__main__":
    args = rospy.myargv(argv=sys.argv)
    if len(args) is 2:
        main(bdaddr = args[1])
    main()
        
