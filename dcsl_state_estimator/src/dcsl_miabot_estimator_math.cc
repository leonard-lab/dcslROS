// dcsl_miabot_estimator_math.cc

#include "dcsl_miabot_estimator_math.h"
#include <math.h>

void miabot_propagate_state(Vector3d& state, const Vector2d& u, double dt)
{
	// propagate the miabot's state forward in time for dt seconds
	// based on initial position state = [x, y, theta], and constant 
	// applied control u = [v, omega]
	double eps = 0.001;
	double x_plus, y_plus, theta_plus;
	// name incoming variables to make code readable
	double x = state(0);
	double y = state(1);
	double theta = state(2);
	double v = u(0);
	double omega = u(1);

	//Vector3d state_plus;
	
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

void miabot_propagate_covariance(Matrix3d& p, const Vector3d& x, const Vector2d& u, 
	const Matrix3d& q, const Matrix3d& r, double dt)
{
	// Integrate the covariance matrix p forward in time using Euler forward method
	// based on covariance dynamics for Extended Kalman-Bucy Filter
	// described in Stengel's "Optimal Control and Estimation" pg 388

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

void miabot_calculate_filter_gain(Matrix3d& k, const Matrix3d& p, const Matrix3d& r)
{
	// Calculate filter gain according to Extended Kalman-Bucy Filter
	// described in Stengel's "Optimal Control and Estimation" pg 388
	// k = p * h * inverse(h*p*h' + r), where for miabot system h = identity
	Matrix3d temp = p+r;
	k = p*temp.inverse();
}

void miabot_update_state(Vector3d& x, const Matrix3d& k, const Vector3d& z)
{
	// update the state estimate using the optimal filter gain k,
	// state estimate x, and measurement z
	x = x + k*(z - x);
}

void miabot_update_covariance(Matrix3d& p, const Matrix3d& k)
{
	Matrix3d ident3d = Matrix3d::Identity();
	p = (ident3d - k)*p;
}
