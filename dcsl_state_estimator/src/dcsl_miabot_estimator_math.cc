/// \file dcsl_miabot_estimator_math.cc
/// Implements functions for calculating low level miabot control independent of ROS library.
// \author Will Scott
#include "dcsl_miabot_estimator_math.h"
#include <math.h>

/// Function to propagate a robot's state forward in time. Equations are based on the 
/// first-order dynamics of a differential drive robot (such as the Miabot PRO).
/// \param[in,out] state      pose of robot [x, y, theta]
/// \param[in] u              applied control [v, omega]
/// \param[in] dt             how far in time to propagate (sec)
void miabot_propagate_state(Vector3d& state, const Vector2d& u, double dt)
{
	double eps = 0.001;
	double x_plus, y_plus, theta_plus;
	// name incoming variables to make code readable
	double x = state(0);
	double y = state(1);
	double theta = state(2);
	double v = u(0);
	double omega = u(1);

	if (abs(omega) < eps) // check if moving in straight line
	{
		theta_plus = theta; // theta remains constant
        x_plus = x + v*dt*cos(theta);
		y_plus = y + v*dt*sin(theta);
	}
	else
	{
		// Otherwise the miabot will go on a circular path with radius v/omega
		theta_plus = theta + omega*dt;
		double radius = v/omega;
		x_plus = x + radius*(sin(theta_plus) - sin(theta));
		y_plus = y + radius*(cos(theta) - cos(theta_plus));
	}
	state << x_plus, y_plus, theta_plus;
}

/// Integrate the covariance matrix p forward in time using Euler forward method.
/// This functin is based on covariance dynamics for Extended Kalman-Bucy Filter
/// as described in Stengel's "Optimal Control and Estimation" pg 388
/// \param[in,out] p      3x3 matrix of covariance estimates
/// \param[in] x          pose of robot [x, y, theta]
/// \param[in] u          applied control [v, omega]
/// \param[in] q          3x3 matrix of plant disturbance weights
/// \param[in] r          3x3 matrix of sensor noise weights
/// \param[in] dt         how far in time to propagate (sec)
void miabot_propagate_covariance(Matrix3d& p, const Vector3d& x, const Vector2d& u, 
	const Matrix3d& q, const Matrix3d& r, double dt)
{
	// First calculate f, the jacobian matrix of system dynamics at current state x
	Matrix3d f = Matrix3d::Zero();
	f(0,2) = -u(0)*sin(x(2));
	f(1,2) =  u(0)*cos(x(2));
	//f << 0, 0, -u(0)*sin(x(2)), 
	//     0, 0,  u(0)*cos(x(2)), 
	//     0, 0,  0;
	// now step forward one iteration of Euler forward method, over time dt
	p = p + dt*(f*p + p*f.transpose() + q);
}

/// Calculate filter gain according to Extended Kalman-Bucy Filter.
/// Described in Stengel's "Optimal Control and Estimation" pg 388 as
/// k = p * h * inverse(h*p*h' + r), where for miabot system h = identity.
/// \param[out] k         3x3 matrix of filter gain
/// \param[in] p          3x3 matrix of covariance estimates
/// \param[in] r          3x3 matrix of sensor noise weights
void miabot_calculate_filter_gain(Matrix3d& k, const Matrix3d& p, const Matrix3d& r)
{
	Matrix3d temp = p+r;
	k = p*temp.inverse();
}

/// Update the state estimate using the optimal filter gain.
/// \param[in,out] x          estimated pose of robot [x, y, theta]
/// \param[in]     k          3x3 matrix of filter gain
/// \param[in]     z          measured pose of robot [x, y, theta]
void miabot_update_state(Vector3d& x, const Matrix3d& k, const Vector3d& z)
{
	x = x + k*(z - x);
}

/// Update the covariance estimate based on filter gain
/// \param[in,out] p      3x3 matrix of covariance estimates
/// \param[in]     k          3x3 matrix of filter gain
void miabot_update_covariance(Matrix3d& p, const Matrix3d& k)
{
	Matrix3d ident3d = Matrix3d::Identity();
	p = (ident3d - k)*p;
}
