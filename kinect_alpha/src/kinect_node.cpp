#include <ros/ros.h>
<<<<<<< HEAD
#include <kinect_alpha/lib_blob.h>
//#include "lib_blob.h"
=======
#include <iostream>
#include "lib_blob.h"
#include <msg_alpha/BlobDistance.h>
>>>>>>> develop
#include <sensor_msgs/CameraInfo.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/Twist.h>

#include <boost/format.hpp>

<<<<<<< HEAD
// PCL includes
//#include "pcl_ros/point_cloud.h"
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

=======
>>>>>>> develop
// OpenCV includes
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <cv_bridge/cv_bridge.h>
<<<<<<< HEAD
#include <cv_bridge/CvBridge.h>
=======
>>>>>>> develop
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

using std::string;
<<<<<<< HEAD
using sensor_msgs::PointCloud2;
namespace enc = sensor_msgs::image_encodings;

// Shorthand for our point cloud type
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;
=======
namespace enc = sensor_msgs::image_encodings;
>>>>>>> develop

// Global variables here
ros::Publisher             cloud_pub_;
image_transport::Publisher image_pub_;
string window_name_;
cv_bridge::CvImagePtr cv_ptr; //conversion variable for ROS Image to cvImage

class KinectNode {
  public:
    KinectNode();
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);
  cv::Mat detectStrap();
  private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber sub_;
<<<<<<< HEAD

    //image_transport::Publisher image_pub_;
    //msg_alpha::BlobDistance blobDist;
    //ros::Publisher blobPub;
=======
    //image_transport::Publisher image_pub_;
    msg_alpha::BlobDistance blobDist;
    ros::Publisher blobPub;
>>>>>>> develop
    int params[7];
	int dilationIterations;
};

KinectNode::KinectNode():
  it_(nh_)
{
<<<<<<< HEAD

=======
>>>>>>> develop
  ros::NodeHandle private_nh("~");

  private_nh.param("hl",params[0], 0);
  private_nh.param("hh",params[1], 255);
  private_nh.param("sl",params[2], 0);
  private_nh.param("sh",params[3], 255);
  private_nh.param("vl",params[4], 0);
  private_nh.param("vh",params[5], 255);
  private_nh.param("dilationIterations",dilationIterations,10);
  private_nh.param("sliceLength",params[7],5);

<<<<<<< HEAD
  std::cout << params[0] << params[1] << params[2] << params[3] << params[4] << params[5] << std::endl;
  sub_ = it_.subscribe("in_image", 1, &KinectNode::imageCallback, this);

  centroidPub = nh_.advertise<msg_alpha::CentroidPoint>("centroid_point", 1);

=======

  std::cout << params[0] << params[1] << params[2] << params[3] << params[4] << params[5] << std::endl;
  sub_ = it_.subscribe("in_image", 1, &KinectNode::imageCallback, this);
  //image_pub_ = it_.advertise("out_image", 1);

  //blobPub = nh_.advertise<msg_alpha::BlobDistance>("blob_dist",1);
  centroidPub = nh_.advertise<msg_alpha::CentroidPoints>("centroid_pnts",1);
>>>>>>> develop

}

/* Function to receive Kinect Data
   @param The Image from the center
   @type sensor_msg
   @return nothing

 */

void KinectNode::imageCallback(const sensor_msgs::ImageConstPtr& image_msg)
{
<<<<<<< HEAD
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
=======

  //Conver the image from ROS Format to OpenCV format
	try	{
		cv_ptr = cv_bridge::toCvCopy(image_msg, enc::BGR8);
	}
	catch (cv_bridge::Exception& e) {
		ROS_ERROR_STREAM("cv_bridge exception: " << e.what());
		return;
	}
}



/*
  Function to Highlight the path strap from Kinect Data
  @param none
  @type cv::Mat which is an OpenCV Matrix 
  @return an openCV Matrix of the image in black and white -- the path segment is white while the surrondings are black
  
 */
cv::Mat KinectNode::detectStrap()	
{

  cv::Mat output;
  try {
	
	cv::cvtColor(cv_ptr->image, output, CV_BGR2HSV);
	
	cv::Mat temp;

  	//Make a vector of Mats to hold the invidiual B,G,R channels
  	vector<Mat> mats;

  	//Split the input into 3 separate channels
  	split(temp, mats);

	//std::cout << mats.size() << std::endl;
   
	//create the range of HSV values that determine the color we desire to threshold based on launch file params
	cv::Scalar lowerBound = cv::Scalar(params[0],params[2],params[4]);
	cv::Scalar upperBound = cv::Scalar(params[1],params[3],params[5]);

	//threshold the image based on the HSV values
	cv::inRange(output,lowerBound,upperBound,output);

	erode(output, output, Mat());

  	dilate(output, output, Mat(), Point(-1,-1), dilationIterations);

	//	cv::imshow("view",output);
	//	cvWaitKey(5);

	return output;
>>>>>>> develop
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
