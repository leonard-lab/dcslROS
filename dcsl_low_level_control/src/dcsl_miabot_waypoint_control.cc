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

  // arrays to hold info about states and waypoints
  geometry_msgs::PoseArray states;
  geometry_msgs::PoseArray waypoints;
  // message that will be sent
  dcsl_messages::TwistArray velocities;

  MiabotWaypointController(const ros::NodeHandle& node_handle, const int numBots)
    : numRobots(numBots),  n(node_handle), states(), waypoints(), velocities()  {}

  void init()
  {
    // create Publisher object where output will be advertised
    // template type in < > is the message type to be published
    Pub_low_level =  n.advertise<dcsl_messages::TwistArray>("cmd_vel", 1);

    // create Subscriber objects to collect states and new waypoint commands
    Sub_state = n.subscribe("state_estimate",1,&MiabotWaypointController::stateCallback,this);
    Sub_waypoint = n.subscribe("waypoints",1,&MiabotWaypointController::waypointCallback,this);
  }

  void stateCallback(const geometry_msgs::PoseArray::ConstPtr& newStates)
  {
    // when new state estimate is made, compute desired velocity from
    // the most recent waypoint info for each robot
    states = *newStates;
    geometry_msgs::Pose thisPose;
    geometry_msgs::Pose thisWaypoint;
    geometry_msgs::Twist thisVelocity;
    double outputVel[3];


    for (int i = 0; i < numRobots; i++)
    {
    	// compute i-th robot's desired velocity...
      thisPose = states.poses[i];
      double pos[3] = {thisPose.position.x, thisPose.position.y, thisPose.position.z};
      double quat[4] = {thisPose.orientation.w,thisPose.orientation.x,thisPose.orientation.y,thisPose.orientation.z};
      thisWaypoint = waypoints.poses[i];
      double way[3] = {thisWaypoint.position.x, thisWaypoint.position.y, thisWaypoint.position.z};
      
      // call function to compute outputVel = [v,omega]
      miabot_waypoint(outputVel,pos,quat,way);

      // place that into velocities message
      velocities.twists[i].linear.x = outputVel[0];
      velocities.twists[i].angular.z = outputVel[1];
    }
    Pub_low_level.publish(velocities);
  
  }

  void waypointCallback(const geometry_msgs::PoseArray::ConstPtr& newWaypoints)
  {
    // This is called when a new waypoint message comes in from
    // the high level control topic
    // It stores the new waypoint in the waypoint array to use later 
    // when a new state estimate comes in.
    waypoints = *newWaypoints;
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
