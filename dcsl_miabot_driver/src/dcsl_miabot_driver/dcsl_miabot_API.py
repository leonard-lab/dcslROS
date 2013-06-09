#!/usr/lib/env python

## @file
#

## @author Brendan Andrade

import math
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
        pass

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

    
