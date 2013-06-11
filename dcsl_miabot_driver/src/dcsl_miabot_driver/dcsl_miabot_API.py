#!/usr/lib/env python

## @file
#

## @author Brendan Andrade

import math as m
from bluetooth import *


##
#
#
class Miabot(object):
    
    ##
    #
    #
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
        self.socket = BluetoothSocket( RFCOMM )
        self.bdaddr = bdaddr
        
    ##
    #
    #
    def connect(self, port):
        '''
        try:
            self.port = get_available_port( RFCOMM )
        except:
            raise
        '''

        try:
            self.socket.connect((self.bdaddr, port))
        except:
            raise
        self.port = port

    ##
    #
    #
    def close(self):
        self.socket.close()

    ##
    #
    #
    def send(self, message):
        try:
            self.socket.send(message)
        except:
            raise
    ##
    #
    #
    def sendMotorCommand(self, lin_vel, ang_vel):
        # Parameters
        diffConversionFactor = 0.1 # Distance between the wheels in meters
        motorScaleFactor = 1000 # Commands are received in the Twist message in meters and the robot accepts them in mm. This is the scale factor.

        targetTransVel = lin_vel
        targetRotVel = ang_vel

        # Difference in speed between the wheels as this is a differential drive robot
        # rotTerm = targetRotVel / diffConversionFactor
        
        # Motor speeds in mm/s
        # leftVel = int((targetTransVel - rotTerm) * motorScaleFactor)
        # rightVel = int((targetTransVel + rotTerm) * motorScaleFactor)
        
        leftVel = int((targetTransVel - targetRotVel*diffConversionFactor/(2.0*m.pi))*motorScaleFactor)
        rightVel = int((targetTransVel + targetRotVel*diffConversionFactor/(2.0*m.pi))*motorScaleFactor)

        # Create string for commanding wheel speed and write it to the serial port.
        motorCommand = '[=<' + str(leftVel) + '><' + str(rightVel) + '>]' + '\n'
        self.send(motorCommand)

    ##
    #
    #
    def stop(self):
        message = "[s]\n"
        self.send(message)

    ##
    #
    #
    def test(self):
        pass

    
