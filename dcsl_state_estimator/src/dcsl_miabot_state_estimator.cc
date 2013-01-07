#include <Eigen/Dense>
#include <Eigen/StdVector>
#include "ros/ros.h"
#include "dcsl_miabot_estimator_math.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseArray.h"
#include "dcsl_messages/TwistArray.h"

/* dcsl_miabot_state_estimator.cc
   This file defines the state estimation node
   for miabots. */

geometry_msgs::Pose vectorToPose(const Eigen::Vector3d& vec)
{
  // helper function to convert between Eigen::vector and Pose message
  geometry_msgs::Pose pose;
  pose.position.x = vec(0);
  pose.position.y = vec(1);
  pose.orientation.z = vec(2);
  return pose;
}

Eigen::Vector3d PoseToVector(const geometry_msgs::Pose& pose)
{
  // helper function to convert between Pose message and Eigen::Vector
  Eigen::Vector3d vec;
  vec << pose.position.x, pose.position.y, pose.orientation.z;
  return vec;
}

class MiabotStateEstimator
{
public:
  int numRobots;
  ros::NodeHandle n;
  ros::Publisher Pub_state;
  ros::Subscriber Sub_measure;
  ros::Subscriber Sub_control;
private:
  double measureTime;
  double stateTime;
  geometry_msgs::PoseArray state_message;
  std::vector<Eigen::Vector3d> x;
  std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > u;
  std::vector<Eigen::Vector3d> z;
  std::vector<Eigen::Matrix3d> p;
  std::vector<Eigen::Matrix3d> k;
  Eigen::Matrix3d q;
  Eigen::Matrix3d r;
  
public:
  MiabotStateEstimator(const ros::NodeHandle& node_handle, const int numBots) :
      n(node_handle), 
      numRobots(numBots),
      Pub_state(),
      Sub_control(), 
      Sub_measure(),
      measureTime(0.0),
      stateTime(0.0),
      state_message(),
      x(numBots),
      u(numBots),
      z(numBots),
      p(numBots),
      k(numBots),
      q(),
      r()
  {
  }

  void init()
  {
    // create Publisher object where output will be advertised
    // template type in < > is the message type to be published
    Pub_state =  n.advertise<dcsl_messages::TwistArray>("state", 1);

    // create Subscriber objects to collect states and new waypoint commands
    Sub_control = n.subscribe("cmd_vel",    1,&MiabotStateEstimator::controlCallback,this);
    Sub_measure = n.subscribe("measurement",1,&MiabotStateEstimator::measureCallback,this);

    // put initial conditions into state arrays
    // this will change later once ros parameter server is set up
    q = Matrix3d::Identity();
    r = Matrix3d::Identity();
    for (int m = 0; m < numRobots; m++) // loop through robots
    {
      x[m] = Vector3d::Zero();
      u[m] = Vector2d::Zero();
      p[m] = Matrix3d::Constant(1.0);
    }
 }

  void controlCallback(const dcsl_messages::TwistArray::ConstPtr& newVelocities)
  {
    // This is called when a new control  message comes in from
    // the low level control topic, updating the stored info in this node
    for (int m = 0; m < numRobots; m++) // loop through robots
    {
      u[m](0) = newVelocities->twists[m].linear.x;
      u[m](1) = newVelocities->twists[m].angular.z;
    }
  }

  void measureCallback(const geometry_msgs::PoseArray::ConstPtr& newMeasurement)
  {
    // This is called when a new measurement message comes in from
    // the vision tracking node, updating the stored info in this node
    // and calling the kalman filter function to update our estimate
    ros::Time rosMeasureTime = ros::Time::now();
    state_message.header.stamp = rosMeasureTime;
    double newMeasureTime = rosMeasureTime.toSec();
    for (int m = 0; m < numRobots; m++) // loop through robots
    {
      // take measurement out of message
      z[m] = PoseToVector(newMeasurement->poses[m]);
      // perform propagation steps
      x[m] = miabot_propagate_state(x[m], u[m], newMeasureTime - stateTime);
      p[m] = miabot_propagate_covariance(x[m], u[m], p[m], q, r, newMeasureTime - measureTime);
      measureTime = stateTime = newMeasureTime;
      // perform update steps
      k[m] = miabot_calculate_filter_gain(p[m],r);
      x[m] = miabot_update_state(x[m],k[m],z[m]);
      p[m] = miabot_update_covariance(p[m],k[m]);
      // place the updated state into the message to be published
      state_message.poses[m] = vectorToPose(x[m]);
    }
    Pub_state.publish(state_message);
  }

  void propagateState()
  {
    // calculate time elapsed and call state propagation function
    double newTime = ros::Time::now().toSec();
    double dt = newTime - stateTime;
    stateTime = newTime;
    for (int m = 0; m < numRobots; m++) // loop through robots
    {
      x[m] = miabot_propagate_state(x[m], u[m], dt);
    }
  }
};


int main(int argc, char **argv)
{
  // This will be a ROS Parameter eventually
  const int numRobots = 1;

  // initialize the node, with name miabot_waypoint_control
  ros::init(argc, argv, "dcsl_miabot_state_estimator");

  // create the NodeHandle which tells ROS which node this is
  ros::NodeHandle n;

  // create and initialize the controller object
  MiabotStateEstimator mse(n, numRobots);
  mse.init();

  while(ros::ok())
  {
    ros::spinOnce();
    mse.propagateState();
  }

  return 0;
}
