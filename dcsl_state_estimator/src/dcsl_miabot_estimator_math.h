// dcsl_miabot_control_math.h
// functions for calculating low level miabot control independent of ROS
#ifndef DCSL_MIABOT_ESTIMATOR_MATH
#define DCSL_MIABOT_ESTIMATOR_MATH

#include <Eigen/Dense>
using namespace Eigen;

Vector3d miabot_propagate_state(const Vector3d& x, const Vector2d& u, double dt);
Matrix3d miabot_propagate_covariance(const Vector3d& x, const Vector2d& u,
	const Matrix3d& p, const Matrix3d& q, const Matrix3d& r, double dt);
Matrix3d miabot_calculate_filter_gain(const Matrix3d& p, const Matrix3d& r);
Vector3d miabot_update_state(const Vector3d& x, const MatrixXd& k, const Vector3d& z);
Matrix3d miabot_update_covariance(const MatrixXd& p, const MatrixXd& k);


#endif