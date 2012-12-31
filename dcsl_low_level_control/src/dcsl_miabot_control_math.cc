// dcsl_miabot_control_math.cc
// implements functions for calculating low level miabot control independent of ROS
#include "dcsl_miabot_control_math.h"
#include <math.h>

void miabot_waypoint(double* output, double* pos, double* quat, double* waypoint)
{
	// simple controller to bring differential-drive robot to waypoint
	// as described by Brendan Andrade

	double k1 = 1;
	double k2 = 1;
	double cosHalfTheta = quat[0];
	double sinHalfTheta = quat[3];
	double theta = 2.0*atan2(sinHalfTheta,cosHalfTheta);
	
	double distance = sqrt(pow(pos[0]-waypoint[0],2) + pow(pos[1]-waypoint[1],2));
	double phi = atan2(waypoint[1]-pos[1], waypoint[0]-pos[0]);

	// calculate output = [forward velocity "v", angular velocity "omega"]
	output[0] = k1*distance*cos(theta-phi);
	output[1] = k2*sin(theta-phi);
}

