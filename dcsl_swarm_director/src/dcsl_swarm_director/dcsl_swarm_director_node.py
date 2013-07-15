#!/usr/bin/env python

## @file
#  A node that subscribes to the cmd_vel_array topic with TwistArray messages and publishes cmd_velX topics with Twist messages where X is the array index from the TwistArray. 

## @author Brendan Andrade


# Not used with catkin
# import roslib
# roslib.load_manifest('dcsl_swarm_director')

import sys

import rospy
from geometry_msgs.msg import Twist
from dcsl_messages.msg import TwistArray, BelugaArray, belugaInput

## Splits a TwistArray message on one topic into Twist messages on multiple topics
class SwarmDirector:
    
    ## Sets up publishers and subscribers
    #
    # @param nRobots is the number of entries in the TwistArray
    # @param robot_type is an identifier for which robot is in use. 0 for belugas, 1 for miabots
    def __init__(self,nRobots, robot_type):
        self.n = nRobots
        self.publisherArray = []
        
        self.robot_type = robot_type

        if robot_type == 0: # Beluga
            pub_base_start = "robot"
            pub_base_end = "/cmd_inputs"
            pub_type = belugaInput
            sub_name = "cmd_input_array"
            sub_type = BelugaArray
        elif robot_type == 1: # Miabot
            pub_base_start = "cmd_vel"
            pub_base_end = ""
            pub_type = Twist
            sub_name = "cmd_vel_array"
            sub_type = TwistArray

        for i in range(0,self.n):
            name = pub_base_start + str(i) + pub_base_end
            publisher = rospy.Publisher(name, pub_type)
            self.publisherArray.append(publisher)
        self.cmdArray_sub = rospy.Subscriber(sub_name, sub_type, self.callback)
        
    ## Callback function for the TwistArray 
    #
    # Routes each Twist in the TwistArray to the appropriate topic
    def callback(self,data):
        for i in range(0,self.n):
            if self.robot_type == 0:
                command = data.belugas[i]
            elif self.robot_type == 1:
                command = data.twists[i]
            self.publisherArray[i].publish(command)

## Main function which is called when the node begins
# 
# Receives the "/n_robots" parameter. Initializes the node and creates the SwarmDirector object.
def main(argv):
    if argv[0] == '--beluga':
        robot_type = 0;
    elif argv[0] == '--miabot':
        robot_type = 1;
    else:
        rospy.logerr("Swarm director node must be given '--beluga' or '--miabot' argument")

    nRobots = rospy.get_param("/n_robots",1)
    rospy.init_node('dcsl_swarm_director')
    director = SwarmDirector(nRobots, robot_type)

    try: 
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"

if __name__ == '__main__':
    main(sys.argv[1:])
