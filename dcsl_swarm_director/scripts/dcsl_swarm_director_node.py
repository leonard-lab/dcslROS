#!/usr/bin/env python

## @file
#  A node that subscribes to the cmd_vel_array topic with TwistArray messages and publishes cmd_velX topics with Twist messages where X is the array index from the TwistArray. 

## @author Brendan Andrade



import roslib
roslib.load_manifest('dcsl_swarm_director')
import rospy
from geometry_msgs.msg import Twist
from dcsl_messages.msg import TwistArray

## Splits a TwistArray message on one topic into Twist messages on multiple topics
class SwarmDirector:
    
    ## Sets up publishers and subscribers
    # @param nRobots is the number of entries in the TwistArray
    def __init__(self,nRobots):
        self.n = nRobots
        self.publisherArray = []
        for i in range(0,self.n):
            name = "cmd_vel"+str(i)
            publisher = rospy.Publisher(name,Twist)
            self.publisherArray.append(publisher)
        self.cmdArray_sub = rospy.Subscriber("cmd_vel_array",TwistArray, self.callback)
        
    ## Callback function for the TwistArray 
    # Routes each Twist in the TwistArray to the appropriate topic
    def callback(self,data):
        for i in range(0,self.n):
            command = data.twists[i]
            self.publisherArray[i].publish(command)

## Main function which is called when the node begins
# 
# Receives the "/n_robots" parameter. Initializes the node and creates the SwarmDirector object.
def main():
    nRobots = rospy.get_param("/n_robots",1)
    rospy.init_node('dcsl_swarm_director')
    director = SwarmDirector(nRobots)

    try: 
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"

if __name__ == '__main__':
    main()
