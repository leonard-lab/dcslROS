//Much of this code was copied from the ROS Tutorial for the joy package 

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

/// \file
/// \brief A node that subscribes to the joy topic, converts the joystick inputs to velocity and angular velocity commands and publishes them on cmd_vel as a Twist.

//\author: Brendan Andrade

///\brief Reads joystick commands from joy topic and publishes corresponding commands for Miabot as Twist on cmd_vel topic
class TeleopMiabot
{
public:
  TeleopMiabot();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::NodeHandle nh_;

  int linear_, angular_;
  double l_scale_, a_scale_;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;

};

/// \brief Initializer. Advertises and subscribes to topics and gathers ROS parameter values
TeleopMiabot::TeleopMiabot():
  linear_(1),
  angular_(2)
{
  //Read parameters
  nh_.param("axis_linear", linear_, linear_);
  nh_.param("axis_angular", angular_, angular_);
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);

  //Advertise topic
  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel",1);

  //Subscribe to joy
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopMiabot::joyCallback, this);

}

/// \brief Triggered on joy callback. Converts joy values to velocity and angular velocity and publishes Twist on cmd_vel
void TeleopMiabot::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  geometry_msgs::Twist cmd_vel;
  cmd_vel.angular.z = a_scale_*joy->axes[angular_];
  cmd_vel.linear.x = l_scale_*joy->axes[linear_];
  vel_pub_.publish(cmd_vel);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_miabot");
  TeleopMiabot teleop_miabot;

  ros::spin();
}
