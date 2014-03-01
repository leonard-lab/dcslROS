#!/usr/lib/env python

## @file dcsl_miabot_API.py Contains the Miabot class for connecting and commanding a Merlin MiabotPro robot over a Bluetooth connection.
#

## @author Brendan Andrade

import math as m
from bluetooth import *


## This object allows easy interaction with a Merlin MiabotPro over a Bluetooth serial connection.
#
# It supports opening and closing of the connection, sending strings, sending linear and angular velocity commands, and stop.
class Miabot(object): 
    
    ## Establishes the identity of the robot by supplying bdaddr or by scanning to find one.
    #
    # @param bdaddr is an optional bluetooth address for the robot
    def __init__(self, bdaddr = None):
        if bdaddr == None:
            try:
                nearby_bdaddrs = discover_devices()
            except:
                raise
            if len(nearby_bdaddrs) > 0:
                    bdaddr = nearby_bdaddrs[0]
            else:
                raise Exception("Miabot initialization error: No bdaddr provided and no miabot found.")
        
        self._bdaddr = bdaddr
        
    ## Opens the Bluetooth Socket to the robot on the supplied port.
    #
    # @param port on which the bluetooth socket should connect
    def connect(self, port):
        
        #try:
        #    self.port = get_available_port( RFCOMM )
        #except:
        #    raise
        
        self._socket = BluetoothSocket( RFCOMM )
        print "Attempting connection to " + self._bdaddr + " on port " + str(port) + "."
        try:
            self._socket.connect((self._bdaddr, port))
        except:
            raise
        self._port = port

    ## Closes the Bluetooth Socket
    #
    #
    def close(self):
        self._socket.close()

    ## Sends a message to the robot over the Bluetooth socket
    #
    # @param message The message (usually a string) to send to the robot.
    def send(self, message):
        try:
            self._socket.sendall(message)
        except:
            raise
    ## Commands the robot to uses its internal feedback loop to move at a specific linear and angular velocity.
    #
    # @param lin_vel the linear velocity to move at in m/s
    # @param ang_vel the angular velocity to rotate at in radians/s
    def sendMotorCommand(self, lin_vel, ang_vel):
        # Parameters
        diffConversionFactor = 0.0667 # Distance between the wheels in meters
        motorScaleFactor = 501 # 1000 # Commands are received in the Twist message in meters and the robot accepts them in mm. This is the scale factor.
        max_motor_speed = 1000

        targetTransVel = lin_vel
        targetRotVel = ang_vel

        # Difference in speed between the wheels as this is a differential drive robot
        # rotTerm = targetRotVel / diffConversionFactor
        
        # Motor speeds in mm/s
        # leftVel = int((targetTransVel - rotTerm) * motorScaleFactor)
        # rightVel = int((targetTransVel + rotTerm) * motorScaleFactor)
        
        leftVel = int((targetTransVel - targetRotVel*diffConversionFactor/(2.0))*motorScaleFactor)
        rightVel = int((targetTransVel + targetRotVel*diffConversionFactor/(2.0))*motorScaleFactor)

        if leftVel > max_motor_speed:
            leftVel = max_motor_speed
        elif leftVel < -1*max_motor_speed:
            leftVel = -1*max_motor_speed

        if rightVel > max_motor_speed:
            rightVel = max_motor_speed
        elif rightVel < -1*max_motor_speed:
            rightVel = -1*max_motor_speed

        # Create string for commanding wheel speed and write it to the serial port.
        motorCommand = '[=<' + str(leftVel) + '>,<' + str(rightVel) + '>]' + '\n'
        self.send(motorCommand)

    ## Sends a stop motors command to the robot
    #
    #
    def stop(self):
        message = "[s]\n"
        self.send(message)

    ##
    #
    #
    # def test(self):
        # pass
