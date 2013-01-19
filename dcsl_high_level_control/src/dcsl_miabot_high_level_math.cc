/// \file dcsl_miabot_high_level_math.cc
/// Implements functions for calculating high level miabot control independent of ROS library.
// \author Will Scott
#include "dcsl_miabot_high_level_math.h"
#include <math.h>

/// Control law to calculate waypoints to cause (i+1)th robot to follow ith robot.
/// Robot (i+1)'s waypoint is placed on the line connecting it with robot i's, set
/// at min_dist = 0.2 from robot i. If the two robots are withing min_dist, then
/// waypoint is set to current position, with orientation facing ith robot.
/// It is assumed that robot 0 is being controlled by joystick, so its waypoint
/// is set to its current position.
/// \param[out] way          waypoint output = [x_0, y_0, theta_0, x_1, y_1, theta_1, ...]
/// \param[in]  pos          pose input      = [x_0, y_0, theta_0, x_1, y_1, theta_1, ...]
/// \param[in]  numRobots    number of robots (length of arrays above are both 3*numRobots)
void miabot_follower_control(double* way, double* pos, int numRobots)
{
	way[0] = pos[0];
	way[1] = pos[1];
	way[2] = pos[2];
	

	if (numRobots > 1)
	{
		double min_dist = 0.2; // radius of target within which to stop
		double dist, phi, this_x, this_y, prev_x, prev_y;
		for (int i = 1; i < numRobots; i++)
		{
			// set waypoint along the vector from bot i to bot i-1,
			// at distance min_dist from bot i-1 (so they don't collide)
			this_x = pos[3*i + 0];
			this_y = pos[3*i + 1];
			//this_theta = pos[3*i + 2];
			prev_x = pos[3*(i-1) + 0];
			prev_y = pos[3*(i-1) + 1];
			//prev_theta = pos[3*(i-1) + 2];
			
			// calculate distance from and angle towards prev bot
			dist = sqrt(pow(this_x-prev_x,2) + pow(this_y-prev_y,2));
			phi = atan2(prev_y - this_y, prev_x - this_x);

			if (dist < min_dist)
			{
				// just stay where it is if it's already close,
				// and turn towards prev bot
				way[3*i + 0] = this_x;
				way[3*i + 1] = this_y;
				way[3*i + 2] = phi;
			}
			else
			{
				// move closer
				way[3*i + 0] = this_x + cos(phi)*(dist - min_dist);
				way[3*i + 1] = this_y + sin(phi)*(dist - min_dist);
				way[3*i + 2] = phi;				
			}

		}
	}
}
