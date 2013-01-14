#include "dcsl_miabot_high_level_math.h"
#include <math.h>

void miabot_follower_control(double* way, double* pos, int numRobots)
{
	// calculate waypoints to cause (i+1)th robot to follow ith robot
	// robot 0 will be controlled by joystick (handled in launch file)
	// so we'll just set it's waypoint to its current position,
	// so the estimator doesn't get too confused (hopefully)
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