#include "ros/ros.h"
#include "ros/console.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "dcsl_messages/TwistArray.h"
#include "dcsl_messages/StateArray.h"
#include "dcsl_miabot_control_math.h"

/// \file dcsl_miabot_low_level_control.cc
/// This file defines the low_level_control node for miabot waypoint control.

// \author Will Scott

/// MiabotLowLevelController class handles callbacks for miabot_low_level_control node
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
  std::vector<double> waypoints_x;
  std::vector<double> waypoints_y;
  std::vector<double> waypoints_theta;
  const double k1;
  const double k2;
  
public:
  /// Constructor for MiabotLowLevelController.
  /// \param[in] node_handle         identifies which node we are running, used to create subscribers/publishers
  /// \param[in] numBots             number of robots to be controlled
  /// \param[in] k1                  forward velocity gain for waypoint control
  /// \param[in] k2                  angular velocity gain for waypoint control
  MiabotLowLevelController(const ros::NodeHandle& node_handle, 
    const int numBots, const double k1, const double k2) :
      numRobots(numBots),
      n(node_handle),
      Pub_low_level(),
      Sub_state(),
      Sub_waypoint(),
      Sub_velocity(),
      waypoints_x(numBots),
      waypoints_y(numBots),
      waypoints_theta(numBots),
      k1(k1),
      k2(k2)  
  {}

  /// Initialization for MiabotLowLevelController, to be called directly after object creation.
  /// Here we initialize subscriber and publisher objects which connect this node to roscore
  /// \param[in] use_waypoint        When true, subscribers are created to listen on "waypoint_input" 
  ///                                and "state_estimate" topics. When false, subscriber is created
  ///                                for "velocity_input"
  void init()
  {
    // create Publisher object where output will be advertised
    // template type in < > is the message type to be published
    Pub_low_level =  n.advertise<dcsl_messages::TwistArray>("cmd_vel_array", 1);

    // create Subscriber objects to collect states and new waypoint commands
    Sub_state    = n.subscribe("state_estimate",1,&MiabotLowLevelController::stateCallback,this);
    Sub_waypoint = n.subscribe("waypoint_input",1,&MiabotLowLevelController::waypointCallback,this);

    // create subscriber for velocity inputs
    Sub_velocity = n.subscribe("velocity_input",1,&MiabotLowLevelController::velocityCallback,this);
  }

  /// Callback function for topic "state_estimate".
  /// This function is called whenever a new message is posted to the "state_estimate" topic.
  /// The new message is used in computing velocities towards the latest waypoints, which are
  /// published to the topic "cmd_vel_array" as a TwistArray
  /// \param[in] states          PoseArray of robot states containing position and heading of each robot.
  void stateCallback(const dcsl_messages::StateArray data)
  {
    // when new state estimate is made, compute desired velocity from
    // the most recent waypoint info for each robot
    ROS_DEBUG_STREAM("received state message");
    if(int(data.states.size()) == numRobots)
    {
      dcsl_messages::TwistArray velocities;
      geometry_msgs::Twist currentTwist;
      double pos[3];
      double way[3];
      double outputVel[2];

      for (int i = 0; i < numRobots; i++) // loop through the robots
      {
      	pos[0] = data.states[i].pose.position.x;
        pos[1] = data.states[i].pose.position.y; 
        pos[2] = data.states[i].pose.orientation.z;
        way[0] = waypoints_x[i];
        way[1] = waypoints_y[i];
        way[2] = waypoints_theta[i];
        
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

  /// Callback function for topic "waypoint_input".
  /// This function is called whenever a new message is posted to the "waypoint_input" topic.
  /// The new message is stored for later use in computing velocities when a new state arrives.
  /// \param[in] newWaypoints          PoseArray of desired robot states.
  void waypointCallback(const geometry_msgs::PoseArray newWaypoints)
  {
    ROS_DEBUG_STREAM("received waypoint message");
    if(int(newWaypoints.poses.size()) == numRobots)
    {
      for (int i = 0; i < numRobots; i++) // loop through the robots
      {
        waypoints_x[i]     = newWaypoints.poses[i].position.x;
        waypoints_y[i]     = newWaypoints.poses[i].position.y;
        waypoints_theta[i] = newWaypoints.poses[i].orientation.z;
      }
    }
    else
    {
      ROS_ERROR_STREAM("number of waypoints did not match numRobots, skipping...");
    }
  }

  /// Callback function for topic "velocity_input".
  /// This function is called whenever a new message is posted to the "velocity_input" topic.
  /// The incoming message is converted from a PoseArray to a TwistArray and published on topic 
  /// "cmd_vel_array." The conversion is needed due to incompatibility of matlab ipc-bridge with
  /// the custom TwistArray message type.
  /// \param[in] velocityPose          PoseArray of desired robot velocities. Linear velocity is 
  ///                                  stored in position.x of each pose, and angular velocity is
  ///                                  stored in orientation.z of each pose.
 void velocityCallback(const dcsl_messages::TwistArray velocityIn)
  {
    if(int(velocityIn.twists.size()) >= numRobots)
    {
      dcsl_messages::TwistArray velocities;
      geometry_msgs::Twist currentTwist;
      for (int m = 0; m < numRobots; m++)
      {
        currentTwist = velocityIn.twists[m];
        velocities.twists.push_back(currentTwist);
      }
      Pub_low_level.publish(velocities);
    }
    else
    {
      ROS_ERROR_STREAM("number of velocity inputs less than numRobots, skipping...");
    }
  }
};

/// main function called when node is launched. Creates a MiabotLowLevelController object
/// and calls ros::spin() to loop through callbacks until program is interupted
/// Command line options: "--way" (default) for waypoint control
///                       "--vel" for velocity control
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

  // create and initialize the controller object
  MiabotLowLevelController mllc(n, numRobots, k1, k2);
  mllc.init();

  // loop through callback queue until node is closed
  ros::spin();

  return 0;
}
