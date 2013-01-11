// dcsl_miabot_control_math.cc
// implements functions for calculating low level miabot control independent of ROS
#include "dcsl_miabot_control_math.h"
#include <math.h>

void miabot_waypoint(double* output, double* pose, double* waypoint, double k1, double k2)
{
	// simple controller to bring differential-drive robot to waypoint
	// as described by Brendan Andrade
	// inputs: pose = [x, y, theta], waypoint = [x, y, theta]
	// output: output = [v, omega]

	// min distance, where robot is considered 'close enough' to waypoint
	double min_dist = 0.02;

	// calculate distance from bot to waypoint
	double distance = sqrt(pow(pose[0]-waypoint[0],2) + pow(pose[1]-waypoint[1],2));
	// calculate angle of bot relative to line towards waypoint
	double phi = atan2(waypoint[1]-pose[1], waypoint[0]-pose[0]);

	// calculate forward speed v
	output[0] = k1*distance*cos(pose[2]-phi);

	// calculate angular speed omega
	if (distance > min_dist)
	{
		// continue moving towards waypoint
		output[1] = -k2*sin(pose[2]-phi);		
	}
	else
	{
		// spin towards the desired direction
		output[1] = -k2*sin(pose[2]-waypoint[2]);
	}
}

