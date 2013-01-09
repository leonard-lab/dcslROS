#!/usr/bin/env python
import math, serial

import roslib; roslib.load_manifest('dcsl_miabot_driver')
import rospy
from geometry_msgs.msg import Twist

def callback(cmd_vel):
    diffConversionFactor = 0.1
    motorScaleFactor = 1000

    targetTransVel = cmd_vel.linear.x
    targetRotVel = cmd_vel.angular.z

    rotTerm = (math.pi/180.0) * targetRotVel / diffConversionFactor

    leftVel = int((targetTransVel - rotTerm) * motorScaleFactor)
    rightVel = int((targetTransVel + rotTerm) * motorScaleFactor)

    motorCommand = '[=<' + str(leftVel) + '><' + str(rightVel) + '>]' + '\n'
    ser.write(motorCommand)

    

def listener():
    rospy.init_node('dcsl_miabot',anonymous=True)
    rospy.Subscriber("cmd_vel", Twist, callback)
    rospy.spin()

def closePort():
    leftVel = 0
    rightVel = 0
    motorCommand = '[=<' + str(leftVel) + '><' + str(rightVel) + '>]' + '\n'
    ser.write(motorCommand)
    ser.close()

if __name__ == "__main__":
    ser = serial.Serial()
    ser.port = '/dev/rfcomm0' #ROSparam
    ser.baudrate = 19200
    ser.timeout = 600
    ser.bytesize = serial.EIGHTBITS
    ser.stopbits = serial.STOPBITS_ONE
    ser.parity = serial.PARITY_NONE
    ser.open()
    listener()
    rospy.on_shutdown(closePort())
        
