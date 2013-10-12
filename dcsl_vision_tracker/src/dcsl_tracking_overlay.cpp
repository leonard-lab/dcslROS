#include "ros/ros.h"
#include "ros/console.h"
#include "geometry_msgs/PoseArray.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"

#include <vector>
#include <string>
#include <cstdlib>

/// \file dcsl_tracking_overlay.cpp
///

/// \author Brendan Andrade


/// TrackingOverlay class does...
class TrackingOverlay
{
private:
  ros::NodeHandle nh;
  ros::Subscriber sub_image_poses;
  std::vector<image_transport::Subscriber> image_sub_vec;
  std::vector<image_transport::Publisher> image_pub_vec;
  int n_cameras;

public:
  /// Constructor for TrackingOverlay
  /// \param[in] node_handle A ROS node handle object.
  /// \param[in] num_cameras The number of cameras to subscribe to.
  TrackingOverlay(const ros::NodeHandle& node_handle, const int num_cameras)
  {
    nh = node_handle;
    n_cameras = num_cameras;
    image_transport::ImageTransport it(nh);
    
    
    //Initialize subscribers and publishers
    image_sub_vec.resize(n_cameras);
    image_pub_vec.resize(n_cameras);
    for (int i=0; i<n_cameras; ++i)
      {
	std::string beginning = "/camera";
	std::string sub_end = "/image_rect_color";
	std::string pub_end = "/tracked_image";

	// Create a lambda function for create subscriber retaining the correct camera ID number
	auto func = [this, i](const sensor_msgs::ImageConstPtr& msg){ image_callback(msg, i);};

	image_sub_vec[i] = it.subscribe(beginning + std::to_string(i) + sub_end, 1, func);
	image_pub_vec[i] = it.advertise(beginning + std::to_string(i) + pub_end, 1);
      }
  };
    

  /// Destructor for TrackingOverlay
  ~TrackingOverlay(){
  };

private:
  /// Callback function for image subscriber
  /// \param[in] msg A ROS ImageConstPtr. Contains the image data.
  /// \param[in] camera_number The camera ID number of the subscribed stream.
  void image_callback(const sensor_msgs::ImageConstPtr& msg, int camera_number)
  {
    ROS_INFO("%d", camera_number);
  };
  

  /// Callback function for the poses of the tracked robots in image coordinates.
  /// \param[in] data ROS PoseArray object containing the positions of the robots in pixel coordinates.
  void pose_callback(const geometry_msgs::PoseArray data)
  {
    ROS_INFO("Pose callback!");
  };
};



/// main function called when the node is launched.
int main(int argc, char **argv)
{
  ros::init(argc, argv, "dcsl_tracking_overlay");
  
  ros::NodeHandle nh;
  
  int n_cameras;
  if (argc < 2) {
    ROS_ERROR("Tracking overlay node requires exactly 1 argument <num_cameras>.");
    return 0
  }
  else {
    n_cameras = atoi(argv[1]);
    if (n_cameras < 1){
      ROS_ERROR("Argument must be a positive integer.");
    }
  }

  TrackingOverlay tov(nh, n_cameras);
  
  while (ros::ok()){
      ros::spinOnce();
  }

  return 0;

};
