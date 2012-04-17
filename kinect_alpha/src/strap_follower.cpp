#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/CameraInfo.h>
#include <msg_alpha/CentroidPoints.h> // custom message type
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_listener.h>

// STL Data Structures
#include <vector>

#include <boost/format.hpp>

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

// Shorthand for our point cloud type
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;

// Global variables here
ros::Publisher             cloud_pub_;
image_transport::Publisher image_pub_;
string window_name_;
tf::TransformListener* tfl_;
string global_frame_ = "base_link";

geometry_msgs::Point findClosestCentroid(PointCloudXYZRGB &cloud, cv_bridge::CvImagePtr cv_ptr);

void detectStrap(cv_bridge::CvImagePtr cv_ptr, cv::Mat &output, int params[]);

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
	
	// Say you found something interesting at pixel (300,150) in the image and want to know its position in 3D space.  Because the cloud data from the Kinect is organized, you can just pick off the point at (300,150) in the cloud.  Also, draw a red circle over the desired point.
	int row = 150; int col = 300;
	//cv::circle(cv_ptr->image, cv::Point(col,row), 10, CV_RGB(255,0,0));
	
	// Get the corresponding 3D point
	//pcl::PointXYZRGB pcl_pt = cloud.at(col, row);
	//geometry_msgs::Point geom_pt;
	geometry_msgs::Point geom_pt = findClosestCentroid(cloud,cv_ptr);
	//geom_pt.x = pcl_pt.x; geom_pt.y = pcl_pt.y; geom_pt.z = pcl_pt.z;
	
	// the TF package requires inputs to be in the form Stamped<sometype>
	tf::Stamped<tf::Point> geom_pt_tf, temp_pt_tf;
	pointMsgToTF(geom_pt, geom_pt_tf);
	geom_pt_tf.frame_id_ = cloud_msg->header.frame_id;
	geom_pt_tf.stamp_    = cloud_msg->header.stamp;
	
	try {
		tfl_->transformPoint(global_frame_, geom_pt_tf, temp_pt_tf);
	}
	catch(tf::TransformException& ex) {
		ROS_ERROR_STREAM(boost::format("Failed to transform point pose from \"%s\" to \"%s\" frame: %s")
			%geom_pt_tf.frame_id_ %global_frame_ %ex.what());
		return;
	}
	tf::pointTFToMsg(temp_pt_tf, geom_pt);
	
	ROS_INFO_STREAM(boost::format("Pixel (%d,%d) maps to 3D point (%.2f,%.2f,%.2f) in frame = \"%s\"")
		%row %col %geom_pt.x %geom_pt.y %geom_pt.z %global_frame_);
	
	// Show the image.  The window does not update without the cvWaitKey.
	cv::imshow(window_name_.c_str(), cv_ptr->image);
	cvWaitKey(5);
	
	// Publish the modified image
  image_pub_.publish(cv_ptr->toImageMsg());
}

geometry_msgs::Point findClosestCentroid(PointCloudXYZRGB &cloud, cv_bridge::CvImagePtr cv_ptr)
{
  geometry_msgs::Point closestPoint;
  return closestPoint;
}

void detectStrap(cv_bridge::CvImagePtr cv_ptr, cv::Mat &output, int params[])
{
  try
  {
    cv::cvtColor(cv_ptr->image, output, CV_BGR2HSV);
    
    cv::Mat temp;

    // Make a vector of Mats to hold the individual B,G,R channels
    std::vector<cv::Mat> mats;

    // Split the input into 3 separate channels
    split(temp,mats);

    // create the range of HSV values that determine the color we desire to threshold based on launch file params
    cv::Scalar lowerBound = cv::Scalar(params[0],params[2],params[4]);
    cv::Scalar upperBound = cv::Scalar(params[1],params[2],params[5]);

    //threshold the image based on the HSV values
    cv::inRange(output,lowerBound,upperBound,output);

    cv::erode(output, output, cv::Mat());
    cv::dilate(output, output, cv::Mat(), cv::Point(-1,-1), params[6]);
  }
  catch(cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert to 'bgr8'. Ex was %s", e.what());
  }
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "rate_limiter");
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");
	image_transport::ImageTransport it(nh);
	
	tf::TransformListener tfl;
	tfl_ = &tfl;
	
	// Get some parameters (optional)
	string mystringparam;
	double mydoubleparam;
	private_nh.param("test" , mystringparam, string("this is a default value"));
	private_nh.param("test2", mydoubleparam, 1.0);
	ROS_INFO_STREAM("mystringparam = " << mystringparam);
	ROS_INFO_STREAM("mydoubleparam = " << mydoubleparam);
	
	private_nh.param("global_frame" , global_frame_, string("this is a default value"));
	ROS_INFO_STREAM("Using global frame \""<<global_frame_<<"\"");
	
	// Subscribe to an image, cloud, and camera info.
	// Note the use of image_transport::SubscriberFilter and message_filters::Subscriber.  These allow for synchronization of the topics.
	image_transport::SubscriberFilter                    image_sub   (it, "in_image", 1);
	message_filters::Subscriber<sensor_msgs::PointCloud2>        cloud_sub   (nh, "in_cloud", 1);
	message_filters::Subscriber<sensor_msgs::CameraInfo> cam_info_sub(nh, "in_cam_info", 1);
	
	// This sync policy will invoke a callback if it receives one of each message with matching timestamps
	typedef message_filters::sync_policies::ApproximateTime
	   <sensor_msgs::Image, sensor_msgs::PointCloud2, sensor_msgs::CameraInfo> MySyncPolicy;
	
	// Synchronize the three topics.  MySyncPolicy(10) tells it to maintain a buffer of 10 messages.
	message_filters::Synchronizer<MySyncPolicy>
	   sync(MySyncPolicy(10), image_sub, cloud_sub, cam_info_sub);
	
	// Hook the callback into the sync policy
	sync.registerCallback( boost::bind(&allCB, _1, _2, _3) );
	
	// publishers for the image and point cloud
  image_pub_ = it.advertise                          (ros::this_node::getName() + "/out_image", 1);
	cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>(ros::this_node::getName() + "/out_cloud", 1);

	window_name_ = "Image from Kinect";
	cv::namedWindow(window_name_.c_str());
	ROS_INFO("Done initializing, going into spin mode.");

	while( ros::ok() )
	{
  	ros::spinOnce();
  }
  
	return 0;
}
