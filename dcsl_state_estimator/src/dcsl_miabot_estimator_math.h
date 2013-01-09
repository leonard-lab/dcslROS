// dcsl_miabot_control_math.h
// functions for calculating low level miabot control independent of ROS
#ifndef DCSL_MIABOT_ESTIMATOR_MATH
#define DCSL_MIABOT_ESTIMATOR_MATH

#include <Eigen/Dense>
using namespace Eigen;

void miabot_propagate_state(Vector3d& x, const Vector2d& u, double dt);
void miabot_propagate_covariance(Matrix3d& p, const Vector3d& x, const Vector2d& u,
	const Matrix3d& q, const Matrix3d& r, double dt);
void miabot_calculate_filter_gain(Matrix3d& k, const Matrix3d& p, const Matrix3d& r);
void miabot_update_state(Vector3d& x, const Matrix3d& k, const Vector3d& z);
void miabot_update_covariance(Matrix3d& p, const Matrix3d& k);


#endif
