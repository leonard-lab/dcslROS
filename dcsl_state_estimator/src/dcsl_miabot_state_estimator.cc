#include <iostream>
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

void vectorToPose(const Eigen::Vector3d& vec, geometry_msgs::Pose& pose)
{
  // helper function to convert between Eigen::vector and Pose message
  pose.position.x = vec(0);
  pose.position.y = vec(1);
  pose.orientation.z = vec(2);
}

void PoseToVector(const geometry_msgs::Pose& pose, Eigen::Vector3d& vec)
{
  // helper function to convert between Pose message and Eigen::Vector3d
  vec << pose.position.x, pose.position.y, pose.orientation.z;
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
  std::vector<Eigen::Vector3d> x;
  std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > u;
  std::vector<Eigen::Vector3d> z;
  std::vector<Eigen::Matrix3d> p;
  std::vector<Eigen::Matrix3d> k;
  Eigen::Matrix3d q;
  Eigen::Matrix3d r;
  
public:
  MiabotStateEstimator(const ros::NodeHandle& node_handle, const int numBots) :
      numRobots(numBots),
      n(node_handle), 
      Pub_state(),
      Sub_measure(),
      Sub_control(), 
      measureTime(0.0),
      stateTime(0.0),
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
    Pub_state =  n.advertise<geometry_msgs::PoseArray>("state_estimate", 1);

    // create Subscriber objects to collect states and new waypoint commands
    Sub_control = n.subscribe("cmd_vel_array",      1,&MiabotStateEstimator::controlCallback,this);
    Sub_measure = n.subscribe("planar_measurements",1,&MiabotStateEstimator::measureCallback,this);

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

  void controlCallback(const dcsl_messages::TwistArray newVelocities)
  {
    // This is called when a new control  message comes in from
    // the low level control topic, updating the stored info in this node
    std::cout << "received new control message" << std::endl;
    for (int m = 0; m < numRobots; m++) // loop through robots
    {
      u[m](0) = newVelocities.twists[m].linear.x;
      u[m](1) = newVelocities.twists[m].angular.z;
    }
  }

  void measureCallback(const geometry_msgs::PoseArray newMeasurement)
  {
    // This is called when a new measurement message comes in from
    // the vision tracking node, updating the stored info in this node
    // and calling the kalman filter function to update our estimate
    ros::Time rosMeasureTime = ros::Time::now();
    double newMeasureTime = rosMeasureTime.toSec();
    
    geometry_msgs::PoseArray state_message;
    state_message.header.stamp = rosMeasureTime;
    geometry_msgs::Pose current_pose;
    for (int m = 0; m < numRobots; m++) // loop through robots
    {
      // take measurement out of message
      PoseToVector(newMeasurement.poses[m], z[m]);
      // perform propagation steps
      miabot_propagate_state(x[m], u[m], newMeasureTime - stateTime);
      miabot_propagate_covariance(p[m], x[m], u[m], q, r, newMeasureTime - measureTime);
      measureTime = stateTime = newMeasureTime;
      // perform update steps
      miabot_calculate_filter_gain(k[m],p[m],r);
      miabot_update_state(x[m],k[m],z[m]);
      miabot_update_covariance(p[m],k[m]);
      // place the updated state into the message to be published
      vectorToPose(x[m], current_pose);
      state_message.poses.push_back(current_pose);
    }
    Pub_state.publish(state_message);
  }

  void propagateState()
  {
    // calculate time elapsed since last estimate was published, 
    // propagates state forward, and publishes the result
    ros::Time newRosTime = ros::Time::now();
    double newTime = newRosTime.toSec();
    //std::cout << "time is now " << std::setprecision(15) << newTime << std::endl;
    double dt = newTime - stateTime;
    //std::cout << "  " << dt << "seconds from previous" << std::endl;
    stateTime = newTime;

    geometry_msgs::PoseArray state_message;
    state_message.header.stamp = newRosTime;
    geometry_msgs::Pose current_pose;
    for (int m = 0; m < numRobots; m++) // loop through robots
    {
      miabot_propagate_state(x[m], u[m], dt);
      vectorToPose(x[m], current_pose);
      state_message.poses.push_back(current_pose);
    }
    //std::cout << "publishing state at time " << stateTime << std::endl;
    Pub_state.publish(state_message);
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

  ros::Rate looprate(20); // 20 hz
  while(ros::ok())
  {
    ros::spinOnce();
    mse.propagateState();
    looprate.sleep();
  }

  return 0;
}
