#include "../src/dcsl_miabot_estimator_math.h"
#include <gtest/gtest.h>
#define _USE_MATH_DEFINES
#include <math.h>
// above two lines allow us to use M_PI for pi

using namespace Eigen;

TEST(StatePropagation, noTimeElapsed){
	// declare some vectors to send to function
	double dt;
	Vector3d x, x_plus;
	Vector2d u;

	// x shouldn't change for dt = 0
	dt = 0;
	x << 1.0, 2.0, 3.0;
	x_plus = x;
	u << 0.0, 0.0;
	miabot_propagate_state(x, u, dt);
	EXPECT_TRUE(x == x_plus);

	x << 1.0, 2.0, 3.0;
	x_plus = x;
	u << 1.0, 0.0;
	miabot_propagate_state(x, u, dt);
	EXPECT_TRUE(x == x_plus);

	x << 1.0, 2.0, 3.0;
	x_plus = x;
	u << 0.0, 1.0;
	miabot_propagate_state(x, u, dt);
	EXPECT_TRUE(x == x_plus);
}

TEST(StatePropagation, straightLineMotion){
	// declare some vectors to send to function
	double dt;
	Vector3d x, x_plus;
	Vector2d u;

	// test straight line motion
	dt = 1.0;
	u << 1.0, 0.0;
	x << 0.0, 0.0, 0.0;
	x_plus << 1.0, 0.0, 0.0;
	miabot_propagate_state(x, u, dt);
	EXPECT_TRUE(x == x_plus);
	x << 0.0, 0.0, 1.0;
	x_plus << cos(1.0), sin(1.0), 1.0;
	miabot_propagate_state(x, u, dt);
	EXPECT_TRUE(x == x_plus);
	x << 1.0, 1.0, 1.0;
	x_plus << 1.0+cos(1.0), 1.0+sin(1.0), 1.0;
	miabot_propagate_state(x, u, dt);
	EXPECT_TRUE(x == x_plus);

 }

TEST(StatePropagation, spinningInPlace){
	// declare some vectors to send to function
	double dt;
	Vector3d x, x_plus;
	Vector2d u;

	// test for v = 0, omega !=0
	dt = 1.0;
	u << 0.0, 1.0;
	x << 0.0, 0.0, 0.0;
	x_plus << 0.0, 0.0, 1.0;
	miabot_propagate_state(x, u, dt);
	EXPECT_TRUE(x == x_plus);
	
	u << 0.0, -1.0;
	x << 0.0, 0.0, 0.0;
	x_plus << 0.0, 0.0, -1.0;
	miabot_propagate_state(x, u, dt);
	EXPECT_TRUE(x == x_plus);

	u << 0.0, 1.0;
	x << 1.0, 2.0, 0.0;
	x_plus << 1.0, 2.0, 1.0;
	miabot_propagate_state(x, u, dt);
	EXPECT_TRUE(x == x_plus);

 }


int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}