#include "ros/ros.h"
#include "ros/console.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "dcsl_miabot_high_level_math.h"

/// \file dcsl_miabot_follower_control.cc
/// This file defines the miabot_follower_control "high level" control node.

// \author Will Scott

/// MiabotFollowerController class handles callbacks for miabot_follower_control node
class MiabotFollowerController
{
public:
  int numRobots;
  ros::NodeHandle n;
  ros::Publisher Pub_waypoint;
  ros::Subscriber Sub_state;
  
public:
  /// Constructor for MiabotFollowerController.
  /// \param[in] node_handle         identifies which node we are running, used to create subscribers/publishers
  /// \param[in] numBots             number of robots to be controlled
  MiabotFollowerController(const ros::NodeHandle& node_handle, const int numBots) :
      numRobots(numBots),
      n(node_handle),
      Pub_waypoint(),
      Sub_state()
  {}

  /// Initialization for MiabotFollowerController, to be called directly after object creation.
  /// Here we initialize subscriber and publisher objects which connect this node to roscore
  void init()
  {
    // create Subscriber and Publisher objects
    Sub_state = n.subscribe("state_estimate",1,&MiabotFollowerController::stateCallback,this);
    Pub_waypoint =  n.advertise<geometry_msgs::PoseArray>("waypoint_input", 1);
  }

  /// Callback function for topic "state_estimate".
  /// This function is called whenever a new message is posted to the "state_estimate" topic.
  /// The new message is used in computing new waypoints based on current positions, using the 
  /// "miabot_follower_control" function in the dcsl_miabot_high_level_math library.
  /// The computed waypoints are published to the topic "waypoint_input" as a PoseArray.
  /// \param[in] states          PoseArray of robot states containing position and heading of each robot.
  void stateCallback(const geometry_msgs::PoseArray states)
  {
    ROS_DEBUG_STREAM("received state message");
    if(int(states.poses.size()) == numRobots)
    {
      geometry_msgs::PoseArray waypoints;
      geometry_msgs::Pose currentPose;
      double pos[3*numRobots];
      double way[3*numRobots];
    

      for (int i = 0; i < numRobots; i++) // loop through the robots
      {
      	// collect all poses into single array
        pos[3*i + 0] = states.poses[i].position.x;
        pos[3*i + 1] = states.poses[i].position.y; 
        pos[3*i + 2] = states.poses[i].orientation.z;
      }
      // call function to compute waypoints as function of current poses
      miabot_follower_control(way,pos,numRobots);
      for (int i = 0; i < numRobots; i++) // loop through the robots
      {
        // insert all waypoints into PoseArray to publish
        currentPose.position.x    = way[3*i + 0];
        currentPose.position.y    = way[3*i + 1]; 
        currentPose.orientation.z = way[3*i + 2];
        waypoints.poses.push_back(currentPose);
      }
      ROS_DEBUG_STREAM("  publishing waypoint message");
      Pub_waypoint.publish(waypoints);
    }
    else
    {
      ROS_ERROR_STREAM("number of states did not match numRobots, skipping...");
    }  
  }
};


/// main function called when node is launched. Creates a MiabotFollowerController object
/// and loops through callbacks at a rate of 5 Hz until program is interupted.
int main(int argc, char **argv)
{
  // initialize the node, with name miabot_follower_control
  ros::init(argc, argv, "miabot_follower_control");

  // create the NodeHandle which tells ROS which node this is
  ros::NodeHandle n;

  // collect number of robots from parameter server (default 1)
  int numRobots;
  n.param<int>("/n_robots", numRobots, 1);

  // create and initialize the controller object
  MiabotFollowerController mfc(n, numRobots);
  mfc.init();

  // loop through callback queue until node is closed
  ros::Rate looprate(5); // 5 hz
  while(ros::ok())
  {
    ros::spinOnce();
    looprate.sleep();
  }


  return 0;
}
