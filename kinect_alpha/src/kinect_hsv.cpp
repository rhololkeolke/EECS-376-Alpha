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

// PCL includes
#include "pcl_ros/point_cloud.h"
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

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

// A magical callback that combines an image, cam info, and point cloud
void allCB(const sensor_msgs::ImageConstPtr& image_msg, 
           const sensor_msgs::PointCloud2::ConstPtr& cloud_msg,
           const sensor_msgs::CameraInfo::ConstPtr& cam_msg)
{
  tfl = new tf::TransformListener();
  string global_frame_ = "map";

  // Convert the image from ROS format to OpenCV format
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(image_msg);
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR_STREAM("cv_bridge exception: " << e.what());
    return;
  }
  
  //ROS_INFO_STREAM(boost::format("Callback got an image in format %s, size %dx%d")
  //  %cv_ptr->encoding %cv_ptr->image.size().width %cv_ptr->image.size().height );

  PointCloudXYZRGB cloud;
  pcl::fromROSMsg(*cloud_msg, cloud);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr originalCloud = cloud;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredRCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredGCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredColorCloud (new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::PassThrough<pcl::PointXYZRGB> Rpass;
  Rpass.setInputCloud (originalCloud);
  Rpass.setFilterFieldName ("r");
  Rpass.setFilterLimits (253, 255);
  Rpass.filter (*filteredRCloud);

  pcl::PassThrough<pcl::PointXYZRGB> Gpass;
  Gpass.setInputCloud (filteredRCloud);
  Gpass.setFilterFieldName ("g");
  Gpass.setFilterLimits (253, 255);
  Gpass.filter (*filteredGCloud);

  pcl::PassThrough<pcl::PointXYZRGB> Bpass;
  Bpass.setInputCloud (filteredGCloud);
  Bpass.setFilterFieldName ("b");
  Bpass.setFilterLimits (253, 255);
  Bpass.filter (*filteredColorCloud);

  try{
      tfl_->transformPoint(global_frame_,geom_pt_tf,)
  }


  //TODO: REPLACE THE INPUT CLOUD THAT IS TAKEN IN BY THE ZPASS FILTER.
  pcl::PassThrough<pcl::PointXYZRGB> zpass;
  zpass.setInputCloud (NEWCLOUDoriginalCloud);
  zpass.setFilterFieldName ("z");
  zpass.setFilterLimits (zTolLow, zTolHigh);
  zpass.filter (*filteredFinalCloud);

  pointArray[] = new 



  if(pcl_pt.z == 0 & pcl_pt.r == 0 & pcl_pt.g == 0 & pcl_pt.b == 0)
  geometry_msgs::Point geom_pt;
  geom_pt.x = pcl_pt.x; geom_pt.y = pcl_pt.y; geom_pt.z = pcl_pt.z;

  // the TF package requires inputs to be in the form Stamped<sometype>
  tf::Stamped<tf::Point> geom_pt_tf, temp_pt_tf;
  pointMsgToTF(geom_pt, geom_pt_tf);
  geom_pt_tf.frame_id_ = cloud_msg->header.frame_id;
  geom_pt_tf.stamp_ = cloud_msg->header.stamp;

  ROS_INFO_STREAM(boost::format("Cloud has size %dx%d. organized=%s")
    %cloud.width %cloud.height %(cloud.isOrganized() ? "true" : "false") );
  
  // Say you found something interesting at pixel (300,150)
  //in the image and want to know its position in 3D space.
  //Because the cloud data from the Kinect is organized,
  //you can just pick off the point at (300,150) in the cloud.
  //Also, draw a red circle over the desired point.
  //int row = 150; int col = 300;
  //cv::circle(cv_ptr->image, cv::Point(col,row), 10, CV_RGB(255,0,0));
  
  // Get the corresponding 3D point


  //Select only the color corresponding pixels
  pcl:PointXYZRGB 
  
  ROS_INFO_STREAM(boost::format("Pixel (%d,%d) maps to 3D point (%.2f,%.2f,%.2f) in TF frame = \"%s\"")
    %row %col %p.x %p.y %p.z %cloud_msg->header.frame_id);
  
  // Show the image.  The window does not update without the cvWaitKey.
  cv::imshow(window_name_.c_str(), cv_ptr->image);
  cvWaitKey(5);
  
  // Publish the modified image
  image_pub_.publish(cv_ptr->toImageMsg());
}


KinectNode::KinectNode():
  it_(nh_)
{
  ros::NodeHandle private_nh("~");
  private_nh.param("hl",params[0], 0);
  private_nh.param("hh",params[1], 255);
  private_nh.param("hl",params[0], 0);
  private_nh.param("hh",params[1], 255);
  private_nh.param("hl",params[0], 0);
  private_nh.param("hh",params[1], 255);

  std::cout << params[0] << params[1]  << std::endl;
  sub_ = it_.subscribe("in_image", 1, &KinectNode::imageCallback, this);
  //image_pub_ = it_.advertise("out_image", 1);
  //blobPub = nh_.advertise<msg_alpha::BlobDistance>("blob_dist",1);
  centroidPub = nh_.advertise<msg_alpha::CentroidPoint>("centroid_point", 1);
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
}
	
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

  //  cv::imshow("view",output);
  //  cvWaitKey(5);

  return output;
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
