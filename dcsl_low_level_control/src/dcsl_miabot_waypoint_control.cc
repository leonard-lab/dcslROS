#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "dcsl_messages/TwistArray.h"
#include "dcsl_miabot_control_math.h"


/* dcsl_miabot_waypoint_control.cc
   This file defines the low_level_control node
   for miabot waypoint control. */

class MiabotWaypointController
{
public:
  int numRobots;
  ros::NodeHandle n;
  ros::Publisher Pub_low_level;
  ros::Subscriber Sub_state;
  ros::Subscriber Sub_waypoint;

private:
  // store latest waypoint as a vector, ordered with
  // all x values, then all y values, so the waypoint for robot j
  // would be x = waypoints[j], y = waypoints[numRobots+j]
  std::vector<double> waypoints;
  
public:
  MiabotWaypointController(const ros::NodeHandle& node_handle, const int numBots)
    : numRobots(numBots),  n(node_handle), Pub_low_level(), Sub_state(), Sub_waypoint(), waypoints(numBots*2)  {}

  void init()
  {
    // create Publisher object where output will be advertised
    // template type in < > is the message type to be published
    Pub_low_level =  n.advertise<dcsl_messages::TwistArray>("cmd_vel_array", 1);

    // create Subscriber objects to collect states and new waypoint commands
    Sub_state = n.subscribe("state_estimate",1,&MiabotWaypointController::stateCallback,this);
    Sub_waypoint = n.subscribe("waypoint_input",1,&MiabotWaypointController::waypointCallback,this);
  }

  void stateCallback(const geometry_msgs::PoseArray states)
  {
    // when new state estimate is made, compute desired velocity from
    // the most recent waypoint info for each robot
    dcsl_messages::TwistArray velocities;
    geometry_msgs::Twist currentTwist;
    double pos[3];
    double way[2];
    double outputVel[2];


    for (int i = 0; i < numRobots; i++) // loop through the robots
    {
    	pos[0] = states.poses[i].position.x;
      pos[1] = states.poses[i].position.y; 
      pos[2] = states.poses[i].orientation.z;
      way[0] = waypoints[i];
      way[1] = waypoints[numRobots + i];
      
      // call function to compute outputVel = [v,omega]
      miabot_waypoint(outputVel,pos,way);

      // place that into velocities message
      currentTwist.linear.x = outputVel[0];
      currentTwist.angular.z = outputVel[1];
      velocities.twists.push_back(currentTwist);
    }
    Pub_low_level.publish(velocities);
  
  }

  void waypointCallback(const geometry_msgs::PoseArray newWaypoints)
  {
    // This is called when a new waypoint message comes in from
    // the high level control topic
    // It stores the new waypoint in the waypoint array to use later 
    // when a new state estimate comes in.
    for (int i = 0; i < numRobots; i++) // loop through the robots
    {
      waypoints[i]             = newWaypoints.poses[i].position.x;
      waypoints[numRobots + i] = newWaypoints.poses[i].position.y;
    }
  }
};


int main(int argc, char **argv)
{
  // This will be a ROS Parameter eventually
  const int numRobots = 1;

  // initialize the node, with name miabot_waypoint_control
  ros::init(argc, argv, "miabot_waypoint_control");

  // create the NodeHandle which tells ROS which node this is
  ros::NodeHandle n;

  // create and initialize the controller object
  MiabotWaypointController mwc(n, numRobots);
  mwc.init();

  ros::spin();

  return 0;
}
