//Much of this code was copied from the ROS Tutorial for the joy package 

#include <ros/ros.h>
#include <dcsl_messages/belugaInput.h>
#include <dcsl_messages/StateArray.h>
#include <sensor_msgs/Joy.h>

class TeleopBeluga
{
public:
  TeleopBeluga();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  void stateCallback(const dcsl_messages::StateArray::ConstPtr& pose_array);

  ros::NodeHandle nh_;

  int linear_, angular_, vertical_;
  double l_scale_, a_scale_, v_scale_;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;
  ros::Subscriber state_sub_;

  double servo;
  int thrust_motor, vertical_motor;

};

TeleopBeluga::TeleopBeluga():
  linear_(1),
  angular_(2),
  vertical_(3)
{

  nh_.param("axis_linear", linear_, linear_);
  nh_.param("axis_angular", angular_, angular_);
  nh_.param("axis_vertical", vertical_, vertical_);
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);
  nh_.param("scale_vertical", v_scale_, v_scale_);
  

  vel_pub_ = nh_.advertise<dcsl_messages::belugaInput>("cmd_inputs",1);

  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopBeluga::joyCallback, this);

  state_sub_ = nh_.subscribe<dcsl_messages::StateArray>("state_estimate", 10, &TeleopBeluga::stateCallback, this);

  servo = 0.0;
  thrust_motor = 0;
  vertical_motor = 0;

}



void TeleopBeluga::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  //dcsl_messages::belugaInput cmd_inputs;
  servo = a_scale_*double(joy->axes[angular_]);
  thrust_motor = l_scale_*joy->axes[linear_];
  vertical_motor = v_scale_*joy->axes[vertical_];
  //vel_pub_.publish(cmd_inputs);
}

void TeleopBeluga::stateCallback(const dcsl_messages::StateArray::ConstPtr& state_array)
{
  dcsl_messages::belugaInput cmd_inputs;
  cmd_inputs.servo = servo;
  cmd_inputs.thrust_motor = thrust_motor;
  cmd_inputs.vertical_motor = vertical_motor;
  cmd_inputs.header.stamp = ros::Time::now();
  vel_pub_.publish(cmd_inputs);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_beluga");
  TeleopBeluga teleop_beluga;

  ros::spin();
}
