#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_listener.h>

// STL Data Structures
#include <vector>

// Math!
#include <math.h>

// PCL includes
#include "pcl_ros/point_cloud.h"
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// OpenCV includes
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.h>

using std::string;
using sensor_msgs::PointCloud2;

// Shorthand for point cloud
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;

// Global Variables
ros::Publisher obsCloudPub;
tf::TransformListener* tfl;
string global_frame;
string robot_frame;

int hl, hh, sl, sh, vl, vh;
int dilationIterations;
double zTolLow, zTolHigh;




// A magical callback that combines an image, cam info, and point cloud
void allCB(const sensor_msgs::ImageConstPtr& image_msg,
	   const sensor_msgs::PointCloud2::ConstPtr& cloud_msg,
	   const sensor_msgs::CameraInfo::ConstPtr& cam_msg)
{
  // Convert the image from ROS format to OpenCV format
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(image_msg);
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR_STREAM("cv_bridge exception: " << e.what());
    return;
  }

  PointCloudXYZRGB cloud;
  pcl::fromROSMsg(*cloud_msg, cloud);

}

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "obstacle_point_cloud");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  image_transport::ImageTransport it(nh);
  
  tf::TransformListener listener;
  tfl = &listener;

  // Get the parameters from the launch file
  private_nh.param("zTolLow", zTolLow, -0.1);
  private_nh.param("zTolHigh", zTolHigh, 0.1);
  private_nh.param("global_frame", global_frame, string("map"));
  private_nh.param("robot_frame", robot_frame, string("base_link"));

  // naptime
  ros::Rate naptime(20); // will sleep to enforce a rate of 20 Hz
  
  // Point Cloud publisher
  obsCloudPub = nh.advertise<sensor_msgs::PointCloud2> ("obsCloud",1);

  while(ros::ok())
  {
    ros::spinOnce();
    naptime.sleep();
  }
  
  return 0;
}
