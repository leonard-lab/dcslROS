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
    
    _feedback = ConnectMiabotFeedback()
    _result = ConnectMiabotResult()
##
    #
    #
    def __init__(self, name, port, bdaddr = None):
        self._action_name = name
        self.port = port
        self.server = actionlib.SimpleActionServer(self._action_name, ConnectMiabotAction, self.connect, False)
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
            # rospy.logwarn(msg)
        
    ##
    #
    #
    def shutdown(self):
        if self.connected:
            rospy.loginfo("Shutting down miabot...")
            self.miabot.stop()
            self.miabot.close()

    ## 
    #
    #
    def connect(self, goal):
        if goal.connect is True:
            self._feedback.in_progress = True;
            self.server.publish_feedback(self._feedback)
            if self.bdaddr is None:
                rospy.loginfo("Scanning for devices...")
                self.miabot = Miabot()
            else:
                self.miabot = Miabot(bdaddr)
            self.miabot.connect(self.port)
            self.connected = True
            self._feedback.in_progress = False;
            self.server.publish_feedback(self._feedback)
            self._result.connected = True
            self.server.set_succeeded(self._result)
            rospy.loginfo("Connected: " + self._action_name)
        else:
            if self.connected is True:
                self.miabot.close()
            self._result.connected = False
            self.server.set_succeeded(self._result)
            self.connected = False
            rospy.loginfo("Disconnected: " + self._action_name)
        
        
## Main function which is called when the node begins
#
# Initializes the node and creates miabot_node object
def main(port = 1, bdaddr = None):
    rospy.init_node('dcsl_miabot_node',anonymous=True)
    name = rospy.get_name()

    if bdaddr is None:
        miabot_node = MiabotNode(name, port)
    else:
        miabot_node = MiabotNode(name, port, bdaddr = bdaddr)

    rospy.spin()
    rospy.on_shutdown(miabot_node.shutdown())

        
if __name__ == "__main__":
    args = rospy.myargv(argv=sys.argv)
    if len(args) is 2:
        main(bdaddr = args[1])
    main()
        
