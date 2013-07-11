/// \file dcsl_miabot_control_math.cc
/// Implements functions for calculating low level miabot control independent of ROS library.
// \author Will Scott
#include "dcsl_miabot_control_math.h"
#include <math.h>


/// Control law to calculate desired velocity based on current position and desired position.
/// This is a simple controller for differential-drive robots, as described by Brendan Andrade.
/// \param[out] output        [v, omega] 
/// \param[in]  pose          [x, y, theta]
/// \param[in]  waypoint      [x, y, theta]
/// \param[in]  k1            forward gain
/// \param[in]  k2            angular gain
void miabot_waypoint(double* output, double* pose, double* waypoint, double k1, double k2)
{
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
	  if (phi <= 3.141592/2 && phi > -3.141592/2)
	    {
	      output[1] = -k2*sin(pose[2]-phi);		
	    }
	  else
	    {
	      output[1] = k2*sin(pose[2]-phi);
	    }
	}
	else
	{
		// spin towards the desired direction
		output[1] = -k2*sin(pose[2]-waypoint[2]);
	}
}

