// dcsl_miabot_estimator_math.cc

#include "dcsl_miabot_estimator_math.h"
#include <math.h>

Vector3d miabot_propagate_state(const Vector3d& state, const Vector2d& u, double dt)
{
	// propagate the miabot's state forward in time for dt seconds
	// based on initial position state = [x, y, theta], and constant 
	// applied control u = [v, omega]
	double eps = 0.001;
	Vector3d state_plus;
	
	if (abs(u(1)) < eps) // check if moving in straight line
	{
		double x_plus = u(0)*dt*cos(state(2));
		double y_plus = u(0)*dt*sin(state(2));
		state_plus << x_plus, y_plus, state(2);
	}
	else
	{
		// Otherwise the miabot will go on a circular path with radius v/omega
		double theta_plus = state(2) + u(1)*dt;
		double radius = u(0)/u(1);
		double x_plus = state(0) + radius*(sin(theta_plus) - sin(state(2)));
		double y_plus = state(1) + radius*(cos(state(2)) - cos(theta_plus));
		state_plus << x_plus, y_plus, theta_plus;
	}
	
	return state_plus;
}

Matrix3d miabot_propagate_covariance(const Vector3d& x, const Vector2d& u, 
	const Matrix3d& p, const Matrix3d& q, const Matrix3d& r, double dt)
{
	// Integrate the covariance matrix p forward in time using Euler forward method
	// based on covariance dynamics for Extended Kalman-Bucy Filter
	// described in Stengel's "Optimal Control and Estimation" pg 388

	// First calculate f, the jacobian matrix of system dynamics at current state x
	Matrix3d f;
	f << 0, 0, -u(0)*sin(x(2)), 0, 0, u(0)*cos(x(2)), 0, 0, 0;
	// now step forward one iteration of Euler forward method, over time dt
	Matrix3d p_plus = p + dt*(f*p + p*f.transpose() + q);
	return p_plus;
}

Matrix3d miabot_calculate_filter_gain(const Matrix3d& p, const Matrix3d& r)
{
	// Calculate filter gain according to Extended Kalman-Bucy Filter
	// described in Stengel's "Optimal Control and Estimation" pg 388
	// k = p * h * inverse(h*p*h' + r), where for miabot system h = identity
	Matrix3d temp = p+r;
	Matrix3d k = p*temp.inverse();
	return k;
}

Vector3d miabot_update_state(const Vector3d& x, const MatrixXd& k, const Vector3d& z)
{
	// update the state estimate using the optimal filter gain k,
	// state estimate x, and measurement z
	Vector3d x_plus = x * k*(z - x);
	return x_plus;
}

Matrix3d miabot_update_covariance(const MatrixXd& p, const MatrixXd& k)
{
	Matrix3d ident3d = Matrix3d::Identity();
	Matrix3d p_plus = (ident3d - k)*p;
	return p_plus;
}