#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseArray.h"
#include "dcsl_messages/TwistArray.h"

/* dcsl_miabot_velocity_control.cc
   This file defines the low_level_control node
   for miabot velocity control. */

class MiabotVelocityController
{
public:
  int numRobots;
  ros::NodeHandle n;
  ros::Publisher Pub_low_level;
  ros::Subscriber Sub_velocity;

  MiabotVelocityController(const ros::NodeHandle& node_handle, const int numBots)
    : n(node_handle), numRobots(numBots)  {}

  void init()
  {
    // create Publisher object where output will be advertised
    // template type in < > is the message type to be published
    Pub_low_level =  n.advertise<dcsl_messages::TwistArray>("cmd_vel", 100);

    // create Subscriber objects to collect states and new waypoint commands
    Sub_velocity = n.subscribe("velocities",100,&MiabotVelocityController::velocityCallback,this);
 }

  void velocityCallback(const dcsl_messages::TwistArray::ConstPtr& newVelocities)
  {
    // This is called when a new velocity  message comes in from
    // the high level control topic
    // It just publishes that info directly to the miabot driver
    Pub_low_level.publish(newVelocities);
  }
};


int main(int argc, char **argv)
{
  // This will be a ROS Parameter eventually
  const int numRobots = 1;

  // initialize the node, with name miabot_waypoint_control
  ros::init(argc, argv, "miabot_velocity_control");

  // create the NodeHandle which tells ROS which node this is
  ros::NodeHandle n;

  // create and initialize the controller object
  MiabotVelocityController mwc(n, numRobots);
  mwc.init();

  ros::spin();

  return 0;
}
