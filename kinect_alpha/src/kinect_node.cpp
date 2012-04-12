#include <ros/ros.h>
#include <iostream>
#include "lib_blob.h"
#include <msg_alpha/BlobDistance.h>
#include <sensor_msgs/CameraInfo.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/Twist.h>

#include <boost/format.hpp>

// OpenCV includes
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

using std::string;
namespace enc = sensor_msgs::image_encodings;

// Global variables here
ros::Publisher             cloud_pub_;
image_transport::Publisher image_pub_;
string window_name_;

class KinectNode {
  public:
    KinectNode();
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
  private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber sub_;
    //image_transport::Publisher image_pub_;
    msg_alpha::BlobDistance blobDist;
    ros::Publisher blobPub;
    int params[6];
};

KinectNode::KinectNode():
  it_(nh_)
{
  ros::NodeHandle private_nh("~");
  private_nh.param("rh",params[0], 0);
  private_nh.param("rl",params[1], 255);
  private_nh.param("bh",params[2], 0);
  private_nh.param("bl",params[3], 255);
  private_nh.param("gh",params[4], 0);
  private_nh.param("gl",params[5], 255);
  std::cout << params[0] << params[1] << params[2] << params[3] << params[4] << params[5] << std::endl;
  sub_ = it_.subscribe("in_image", 1, &KinectNode::imageCallback, this);
  //image_pub_ = it_.advertise("out_image", 1);
  blobPub = nh_.advertise<msg_alpha::BlobDistance>("blob_dist",1);
}

void KinectNode::imageCallback(const sensor_msgs::ImageConstPtr& image_msg)
{
	// Convert the image from ROS format to OpenCV format
	cv_bridge::CvImagePtr cv_ptr;
	try	{
		cv_ptr = cv_bridge::toCvCopy(image_msg, enc::BGR8);
	}
	catch (cv_bridge::Exception& e) {
		ROS_ERROR_STREAM("cv_bridge exception: " << e.what());
		return;
	}
	
//	ROS_INFO_STREAM(boost::format("Callback got an image in format %s, size %dx%d")
//		%cv_ptr->encoding %cv_ptr->image.size().width %cv_ptr->image.size().height );

  cv::Mat output;
  try {
    //normalizeColors(cv_ptr->image, output);
	int tempInt;
    blobfind(params, cv_ptr->image, output, tempInt);
    //findLines(cv_ptr->image, output);
    cv::imshow("view", output);
    cvWaitKey(5);
    IplImage temp = output;
	blobDist.dist = tempInt;
	std::cout << blobDist << std::endl;
    KinectNode::blobPub.publish(blobDist);
    //image_pub_.publish(bridge.cvToImgMsg(&temp, "bgr8"));
  }
  catch (cv_bridge::Exception& e) {

    ROS_ERROR("Could not convert to 'bgr8'. Ex was %s", e.what());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "kinect_alpha");
  cv::namedWindow("view"); //these cv* calls are need if you want to use cv::imshow anywhere in your program
  cvStartWindowThread();
  KinectNode motion_tracker;
  ros::spin();
  cvDestroyWindow("view");
}
