#!/usr/bin/env python

## @file
# A node that subscribes to the cmd_vel topic with Twist messages. It converts the velocity and angular velocity in the message to wheel speed commands and sends them to the robot over a serial connection.

## @author Brendan Andrade

import math, serial, sys

# Not used with catkin
# import roslib; roslib.load_manifest('dcsl_miabot_driver')

import rospy
from geometry_msgs.msg import Twist


## The callback function for when a Twist message is published on the cmd_vel topic.
#
# This function pulls the commands from the Twist message, converts them into wheel speeds for the robot, and then writes the wheel speed command to the serial port.
# @param cmd_vel is the message object received by the subscriber.
def callback(cmd_vel):
    # Parameters
    diffConversionFactor = 0.1 # Distance between the wheels in meters
    motorScaleFactor = 1000 # Commands are received in the Twist message in meters and the robot accepts them in mm. This is the scale factor.

    # Receive commanded velocity and angular velocity from message
    targetTransVel = cmd_vel.linear.x
    targetRotVel = cmd_vel.angular.z

    # Difference in speed between the wheels as this is a differential drive robot
    rotTerm = (math.pi/180.0) * targetRotVel / diffConversionFactor

    # Motor speeds in mm/s
    leftVel = int((targetTransVel - rotTerm) * motorScaleFactor)
    rightVel = int((targetTransVel + rotTerm) * motorScaleFactor)

    # Create string for commanding wheel speed and write it to the serial port.
    motorCommand = '[=<' + str(leftVel) + '><' + str(rightVel) + '>]' + '\n'
    ser.write(motorCommand)

    
## Main function which is called when the node begins
#
# Initializes the node and the subscriber
def main():
    rospy.init_node('dcsl_miabot',anonymous=True)
    rospy.Subscriber("cmd_vel", Twist, callback)
    rospy.spin()

## This function is triggered when the node shuts down.
#
# Turns off the motor and closes the serial port.
def closePort():
    leftVel = 0
    rightVel = 0
    motorCommand = '[=<' + str(leftVel) + '><' + str(rightVel) + '>]' + '\n'
    ser.write(motorCommand)
    ser.close()

if __name__ == "__main__":
    args = rospy.myargv(argv=sys.argv)
    ser = serial.Serial()
    port = args[1]
    rospy.loginfo("Connecting to serial port at %s", port)
    ser.port = port
    ser.baudrate = 19200
    ser.timeout = 600
    ser.bytesize = serial.EIGHTBITS
    ser.stopbits = serial.STOPBITS_ONE
    ser.parity = serial.PARITY_NONE
    ser.open()
    main()
    rospy.on_shutdown(closePort())
        
