// dcsl_miabot_high_level_math.h
// functions for calculating high level miabot control independent of ROS
#ifndef DCSL_MIABOT_HIGH_LEVEL_MATH
#define DCSL_MIABOT_HIGH_LEVEL_MATH


void miabot_follower_control(double* waypoint, double* pos, int numRobots);


#endif