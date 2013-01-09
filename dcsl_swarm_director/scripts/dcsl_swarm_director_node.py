#!/usr/bin/env python

import roslib
roslib.load_manifest('dcsl_swarm_director')
import rospy
from geometry_msgs.msg import Twist
from dcsl_messages.msg import TwistArray

class SwarmDirector:
    
    def __init__(self,nRobots):
        self.n = nRobots
        self.publisherArray = []
        for i in range(0,self.n):
            name = "cmd_vel"+str(i)
            publisher = rospy.Publisher(name,Twist)
            self.publisherArray.append(publisher)
        self.cmdArray_sub = rospy.Subscriber("cmd_vel_array",TwistArray, self.callback)
        
    def callback(self,data):
        for i in range(0,self.n):
            command = data.twists[i]
            self.publisherArray[i].publish(command)


def main():
    nRobots = 1 #Make ROS param
    rospy.init_node('dcsl_swarm_director')
    director = SwarmDirector(nRobots)

    try: 
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"

if __name__ == '__main__':
    main()
