#!/usr/bin/env python

## @file
# A node that subscribes to the cmd_vel topic with Twist messages. It converts the velocity and angular velocity in the message to wheel speed commands and sends them to the robot over a Bluetooth Socket.
# It serves a ConnectMiabot action for initiating connecting and disconnecting of the robot.
# It can take a YAML dictionary parameter ~miabot_dictionary that assigns bdaddr to ID numbers for each robot.
#
# Usage dcsl_miabot_node.py <bdaddr (optional)>


## @author Brendan Andrade

import sys

# Not used with catkin
# import roslib; roslib.load_manifest('dcsl_miabot_driver')

import actionlib
import rospy
from geometry_msgs.msg import Twist
from dcsl_miabot_driver.msg import *

from dcsl_miabot_API import *


## This class defines a ROS node for interacting with a Merlin MiabotPro robot.
#
# It subscribes to the cmd_vel topic containing Twist messages to receive linear and angular velocity commands.
# It serves a ConnectMiabot action for initiating connecting and disconnecting of the robot.
# It can take a YAML dictionary parameter ~miabot_dictionary that assigns bdaddr to ID numbers for each robot.
class MiabotNode(object):
    
    _feedback = ConnectMiabotFeedback()
    _result = ConnectMiabotResult()

    ## On initialization, an action server is started and subscription to cmd_vel begins.
    #
    # @param name is the name of the node.
    # @param port is the Bluetooth Socket port.
    # @param bdaddr is optional. It's the Bluetooth hardware address of the robot. If none is provided, the node will scan for Bluetooth devices.
    def __init__(self, name, port, bdaddr = None):
        self._action_name = name
        self.port = port
        self.server = actionlib.SimpleActionServer(self._action_name, ConnectMiabotAction, self.connect, False)
        self.server.start()
        self.connected = False
        self.bdaddr = bdaddr
        self.bdaddr_dictionary = rospy.get_param('~miabot_dictionary', None)
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
        
    ## Stop wheel motors and then close Bluetooth Socket to robot.
    #
    #
    def shutdown(self):
        if self.connected:
            rospy.loginfo("Shutting down miabot...")
            self.miabot.stop()
            self.miabot.close()

    ## Callback function for the ConnectMiabot action server.
    #
    # If connect variable of goal is true and miabot_dictionary set and miabot_id variable of goal is set, connect to that robot if not connected. If connect variable true and dictionary not set and miabot_id set, error. If connect var true and no miabot_id set and bdaddr was not provided, scan and connect. If connect true and no miabot_id set and bdaddr provided, connect to that robot. If connect false, disconnect if connected. Otherwise do nothing.
    # @param goal is the goal object sent with the action command.
    def connect(self, goal):
        if goal.connect is True:
            self._feedback.in_progress = True
            self.server.publish_feedback(self._feedback)
            if goal.miabot_id is None:
                if self.bdaddr is None:
                    rospy.loginfo("Scanning for devices...")
                    self.miabot = Miabot()
                else:
                    rospy.loginfo("Connecting to bdaddr: " + str(self.bdaddr))
                    self.miabot = Miabot(self.bdaddr)
            else:
                if self.bdaddr_dictionary is not None:
                    rospy.loginfo("Connecting to Miabot #" + str(goal.miabot_id))
                    self.miabot = Miabot(self.bdaddr_dictionary[str(goal.miabot_id)])
                else:
                    rospy.logerr("Goal contained Miabot ID to connect to but miabot_dictionary parameter was not set.")
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
# @param port is the Bluetooth Socket port to use
# @param bdaddr is optional. The bdaddr of the robot to connect to.
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
        
