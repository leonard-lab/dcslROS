#!/usr/bin/env python

import roslib
roslib.load_manifest('dcsl_swarm_director')
import rospy

from std_msgs.msg import Int16
from std_msgs.msg import Float32

class DepthConverter:

    def __init__(self):
        self.pub = rospy.Publisher("depth_meters", Float32)
        self.sub = rospy.Subscriber("depth_measurement", Int16, self.callback)

    def callback(self, data):
        reading_air = 135
        reading_water = 615
        depth_of_reading_water = 2.0066 #meters
        tank_depth = 2.3876
        depth_meters = tank_depth-(depth_of_reading_water/(reading_water-reading_air)*(data.data-reading_air))
        self.pub.publish(depth_meters);

def main():
    rospy.init_node('depth_converter')
    converter = DepthConverter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"

if __name__ == '__main__':
    main()

    
