#include "ros/ros.h"
#include "ros/console.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "dcsl_messages/TwistArray.h"
#include "dcsl_miabot_control_math.h"


/* dcsl_miabot_waypoint_control.cc
   This file defines the low_level_control node
   for miabot waypoint control. */

class MiabotLowLevelController
{
public:
  int numRobots;
  ros::NodeHandle n;
  ros::Publisher Pub_low_level;
  ros::Subscriber Sub_state;
  ros::Subscriber Sub_waypoint;
  ros::Subscriber Sub_velocity;

private:
  // store latest waypoint as a vector, ordered with
  // all x values, all y values, then all theta values, 
  // so the waypoint for robot j would be x = waypoints[j], 
  // y = waypoints[numRobots+j], and theta = waypoints[numRobots*2 + j]
  std::vector<double> waypoints;
  const double k1;
  const double k2;
  
public:
  MiabotLowLevelController(const ros::NodeHandle& node_handle, 
    const int numBots, const double k1, const double k2) :
      numRobots(numBots),
      n(node_handle),
      Pub_low_level(),
      Sub_state(),
      Sub_waypoint(),
      Sub_velocity(),
      waypoints(numBots*3),
      k1(k1),
      k2(k2)  
  {}

  void init(bool use_waypoint)
  {
    // create Publisher object where output will be advertised
    // template type in < > is the message type to be published
    Pub_low_level =  n.advertise<dcsl_messages::TwistArray>("cmd_vel_array", 1);

    if (use_waypoint)
    {
      // create Subscriber objects to collect states and new waypoint commands
      Sub_state    = n.subscribe("state_estimate",1,&MiabotLowLevelController::stateCallback,this);
      Sub_waypoint = n.subscribe("waypoint_input",1,&MiabotLowLevelController::waypointCallback,this);      
    }
    else
    {
      // create subscriber for velocity inputs
      Sub_velocity = n.subscribe("velocity_input",1,&MiabotLowLevelController::velocityCallback,this);
    }
  }

  void stateCallback(const geometry_msgs::PoseArray states)
  {
    // when new state estimate is made, compute desired velocity from
    // the most recent waypoint info for each robot
    ROS_DEBUG_STREAM("received state message");
    if(int(states.poses.size()) == numRobots)
    {
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
        way[2] = waypoints[numRobots*2 + i];
        
        // call function to compute outputVel = [v,omega]
        miabot_waypoint(outputVel,pos,way,k1,k2);

        // place that into velocities message
        currentTwist.linear.x = outputVel[0];
        currentTwist.angular.z = outputVel[1];
        velocities.twists.push_back(currentTwist);
      }
      ROS_DEBUG_STREAM("  publishing velocity message");
      Pub_low_level.publish(velocities);
    }
    else
    {
      ROS_ERROR_STREAM("number of states did not match numRobots, skipping...");
    }  
  }

  void waypointCallback(const geometry_msgs::PoseArray newWaypoints)
  {
    // This is called when a new waypoint message comes in from
    // the high level control topic
    // It stores the new waypoint in the waypoint array to use later 
    // when a new state estimate comes in.
    ROS_DEBUG_STREAM("received waypoint message");
    if(int(newWaypoints.poses.size()) == numRobots)
    {
      for (int i = 0; i < numRobots; i++) // loop through the robots
      {
        waypoints[i]               = newWaypoints.poses[i].position.x;
        waypoints[numRobots + i]   = newWaypoints.poses[i].position.y;
        waypoints[2*numRobots + i] = newWaypoints.poses[i].orientation.z;
      }
    }
    else
    {
      ROS_ERROR_STREAM("number of waypoints did not match numRobots, skipping...");
    }
  }

 void velocityCallback(const geometry_msgs::PoseArray velocityPose)
  {
    // This is called when a new velocity  message comes in from
    // the high level control topic
    // We move the data from the PoseArray into a TwistArray and publish it
    // (high level control has limitation of not being able to output TwistArray)
    if(int(velocityPose.poses.size()) == numRobots)
    {
      dcsl_messages::TwistArray velocities;
      geometry_msgs::Twist currentTwist;
      for (int m = 0; m < numRobots; m++)
      {
        currentTwist.linear.x = velocityPose.poses[m].position.x;
        currentTwist.angular.z = velocityPose.poses[m].orientation.z;
        velocities.twists.push_back(currentTwist);
      }
      Pub_low_level.publish(velocities);
    }
    else
    {
      ROS_ERROR_STREAM("number of velocity inputs did not match numRobots, skipping...");
    }
  }
};


int main(int argc, char **argv)
{
  // initialize the node, with name miabot_waypoint_control
  ros::init(argc, argv, "miabot_waypoint_control");

  // create the NodeHandle which tells ROS which node this is
  ros::NodeHandle n;

  // collect number of robots from parameter server (default 1)
  int numRobots;
  n.param<int>("/n_robots", numRobots, 1);

  // collect waypoint control gains from parameter server
  double k1, k2;
  n.param<double>("waypoint_gain_1", k1, 0.7);
  n.param<double>("waypoint_gain_2", k2, 0.5);

  // check command line args to see whether to use waypoint or velocity control
  bool use_waypoint;
  if (argc > 1 && std::string(argv[1])=="--way")
  {
    use_waypoint = true;
  }
  else if (argc > 1 && std::string(argv[1])=="--vel")
  {
    use_waypoint = false; // use velocity control
  }
  else
  {
    use_waypoint = true; // default for no flag specified
    ROS_INFO_STREAM("No arg for control type, defaulting to waypoint");
  }

  // create and initialize the controller object
  MiabotLowLevelController mllc(n, numRobots, k1, k2);
  mllc.init(use_waypoint);

  // loop through callback queue until node is closed
  ros::spin();

  return 0;
}
