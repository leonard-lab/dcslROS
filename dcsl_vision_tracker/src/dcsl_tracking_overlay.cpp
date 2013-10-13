#include "ros/ros.h"
#include "ros/console.h"
#include "geometry_msgs/PoseArray.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/imgproc/imgproc.hpp>

#include <vector>
#include <string>
#include <cstdlib>
#include <math.h>

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
  std::vector<double> latest_pose_times;
  std::vector<double> latest_image_times;
  std::vector<const geometry_msgs::PoseArray*> latest_pose_arrays;
  std::vector<cv_bridge::CvImagePtr> latest_images;

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

	// Initialize values of latest_pose_times and latest_image_times
	latest_pose_times.push_back(-1);
	latest_image_times.push_back(-1);
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
    
    //Convert image from message to CvImage
    cv_bridge::CvImagePtr cv_ptr;
    try
      {
	cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
      }
    catch (cv_bridge::Exception& e)
      {
	ROS_ERROR("cv_bridge exception: %s", e.what());
      }
    
    double time = msg->header.stamp.toSec();
    latest_image_times[camera_number] = time;

    // If poses for this image have already been received do overlay and publish
    if (abs(time-latest_pose_times[camera_number]) < 0.005)
      {
	perform_overlay(cv_ptr, *latest_pose_arrays[camera_number]);
	image_pub_vec[camera_number].publish(cv_ptr->toImageMsg());
      }
    // Otherwise store image and pose callback will overlay and publish once pose data arrives
    else
      {
	latest_images[camera_number] = cv_ptr;
      }
  };
  

  /// Callback function for the poses of the tracked robots in image coordinates.
  /// \param[in] data ROS PoseArray object containing the positions of the robots in pixel coordinates.
  void pose_callback(const geometry_msgs::PoseArray& data)
  {
    double time = data.header.stamp.toSec();
    int camera_number = atoi(data.header.frame_id.c_str());
    latest_pose_times[camera_number] = time;

    // If the image for this pose data has arrived, do overlay and publish
    if (abs(time-latest_image_times[camera_number]) < 0.005)
      {
	perform_overlay(latest_images[camera_number], data);
	image_pub_vec[camera_number].publish(latest_images[camera_number]->toImageMsg());
      }
    // Otherwise store pose and image callback will overlay and publish once pose data arrives
    else
      {
	latest_pose_arrays[camera_number] = &data;
      }    
  };


  /// 
  void perform_overlay(cv_bridge::CvImagePtr& image_ptr, const geometry_msgs::PoseArray& pose_array)
  {
    
    int n_robots = pose_array.poses.size();

    double length = 15;
    int radius = 5;
    cv::Scalar red = cv::Scalar(255, 0, 0);
    int text_offset[] = {-30, -20};

    // For each robot draw circle and arrow and label.
    for (int i = 0; i < n_robots; ++i){
      geometry_msgs::Pose pose = pose_array.poses[i];
      if (pose.orientation.w != 1){
	// Robot is not detected in frame so skip to next iteration
	continue;
      }

      cv::Point end = cv::Point(int(pose.position.x + cos(pose.orientation.z)*length), int(pose.position.y - sin(pose.orientation.z)*length));
      cv::Point center = cv::Point(int(pose.position.x), int(pose.position.y));
      cv::Point text_point = cv::Point(int(pose.position.x)-text_offset[0], int(pose.position.y)-text_offset[1]);

      cv::line(image_ptr->image, center, end, red);
      cv::circle(image_ptr->image, center, radius, red, -1);
      cv::putText(image_ptr->image, std::string("Robot ") + std::to_string(i), text_point, cv::FONT_HERSHEY_SIMPLEX, 1.0, red);
    }
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
    return 0;
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
