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
using namespace cv;

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
	int dilationIterations;
};

KinectNode::KinectNode():
  it_(nh_)
{
  ros::NodeHandle private_nh("~");
  private_nh.param("hl",params[0], 0);
  private_nh.param("hh",params[1], 255);
  private_nh.param("sl",params[2], 0);
  private_nh.param("sh",params[3], 255);
  private_nh.param("vl",params[4], 0);
  private_nh.param("vh",params[5], 255);
  private_nh.param("dilationIterations",dilationIterations,10)

  std::cout << params[0] << params[1] << params[2] << params[3] << params[4] << params[5] << std::endl;
  std::cout << dilationIterations << std::endl;
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
	
	cv::cvtColor(cv_ptr->image, output, CV_BGR2HSV);
	
	cv::Mat temp;

  	//Make a vector of Mats to hold the invidiual B,G,R channels
  	vector<Mat> mats;

  	//Split the input into 3 separate channels
  	split(temp, mats);

	//std::cout << mats.size() << std::endl;
   
	cv::Scalar lowerBound = cv::Scalar(params[0],params[2],params[4]);
	cv::Scalar upperBound = cv::Scalar(params[1],params[3],params[5]);

	cv::inRange(output,lowerBound,upperBound,output);

    //Threshold the HSV image where H holds the values, in this case look for the specified lower and upper bounds of orange in the image
    // Set all values below value to zero, leave rest the same
  // Then inverse binary threshold the remaining pixels
  // Threshold blue channel
  //cv::threshold(mats[0], mats[0], params[0], 180, THRESH_TOZERO_INV);
  /*threshold(mats[0], mats[0], params[1], 255, THRESH_BINARY);
  // Threshold green channel
  threshold(mats[1], mats[1], params[2], 255, THRESH_TOZERO_INV);
  threshold(mats[1], mats[1], params[3], 255, THRESH_BINARY);
  // Threshold red channel
  threshold(mats[2], mats[2], params[4], 255, THRESH_TOZERO_INV);
  threshold(mats[2], mats[2], params[5], 255, THRESH_BINARY);
*/
	//multiply(mats[0], mats[1], output);
	//multiply(output, mats[2], output);

    //Convert the black and white image to an RGB(BGR) image for the sake of the point cloud
    //cvtColor(output, output,CV_HSV2BGR);

    //Display the images
	//cv::imshow("view",cv_ptr->image);

	erode(output, output, Mat());

  	//dilate(output, output, Mat(), Point(-1,-1), 30);

	cv::imshow("view",output);
    cvWaitKey(5);





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
