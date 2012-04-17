#include <ros/ros.h>
#include <kinect_alpha/lib_blob.h>
//#include "lib_blob.h"
#include <sensor_msgs/CameraInfo.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/Twist.h>

#include <boost/format.hpp>

// PCL includes
//#include "pcl_ros/point_cloud.h"
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// OpenCV includes
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <cv_bridge/cv_bridge.h>
#include <cv_bridge/CvBridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

using std::string;
using sensor_msgs::PointCloud2;
namespace enc = sensor_msgs::image_encodings;

// Shorthand for our point cloud type
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;

// Global variables here
ros::Publisher             cloud_pub_;
image_transport::Publisher image_pub_;
string window_name_;

// A magical callback that combines an image, cam info, and point cloud
void allCB(const sensor_msgs::ImageConstPtr& image_msg, 
		   const sensor_msgs::PointCloud2::ConstPtr& cloud_msg,
		   const sensor_msgs::CameraInfo::ConstPtr& cam_msg)
{
	// Convert the image from ROS format to OpenCV format
	cv_bridge::CvImagePtr cv_ptr;
	try	{
		cv_ptr = cv_bridge::toCvCopy(image_msg);
	}
	catch (cv_bridge::Exception& e) {
		ROS_ERROR_STREAM("cv_bridge exception: " << e.what());
		return;
	}
	
	ROS_INFO_STREAM(boost::format("Callback got an image in format %s, size %dx%d")
		%cv_ptr->encoding %cv_ptr->image.size().width %cv_ptr->image.size().height );

	// Convert the image from ROS format to PCL format
	PointCloudXYZRGB cloud;
	pcl::fromROSMsg(*cloud_msg, cloud);

	ROS_INFO_STREAM(boost::format("Cloud has size %dx%d. organized=%s")
		%cloud.width %cloud.height %(cloud.isOrganized() ? "true" : "false") );
	
	// Show the image.  The window does not update without the cvWaitKey.
	cv::imshow(window_name_.c_str(), cv_ptr->image);
	cvWaitKey(5);
}

class KinectNode {
  public:
    KinectNode();
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
  private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber sub_;
    image_transport::Publisher image_pub_;
};

KinectNode::KinectNode():
  it_(nh_)
{
  sub_ = it_.subscribe("image", 1, &KinectNode::imageCallback, this);
  image_pub_ = it_.advertise("out_image", 1);
}

void KinectNode::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  sensor_msgs::CvBridge bridge;
  cv::Mat output;
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR_STREAM("cv_bridge exception: " << e.what());
    return;
  }
  try {
    //normalizeColors(image, output);
    //blobfind(image, output);
    findLines(cv_ptr->image, output);
    //cv::imshow("view", output);
    IplImage temp = output;
    image_pub_.publish(bridge.cvToImgMsg(&temp, "bgr8"));
  }
  catch (sensor_msgs::CvBridgeException& e) {

    ROS_ERROR("Could not convert to 'bgr8'. Ex was %s", e.what());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "kinect_alpha");
  KinectNode motion_tracker;
  //cvNamedWindow("view"); //these cv* calls are need if you want to use cv::imshow anywhere in your program
  //cvStartWindowThread();
  ros::spin();
  //cvDestroyWindow("view");
}
