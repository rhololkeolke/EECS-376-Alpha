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
#include <pcl/filters/passthrough.h>

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
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudXYZRGBPtr;

// Global Variables
ros::Publisher obsCloudPub;
tf::TransformListener* tfl;
string global_frame;
string robot_frame;

int hl, hh, sl, sh, vl, vh;
int dilationIterations;
double zLow, zHigh;

// Function prototypes
void detectStrap(cv_bridge::CvImagePtr cv_ptr, cv::Mat &output);

PointCloudXYZRGB::Ptr filterCloud(cv::Mat &input, PointCloudXYZRGB &cloud, string frame_id, ros::Time stamp);

pcl::PointXYZRGB transformPoint(pcl::PointXYZRGB pcl_pt, string target_frame, string cloud_frame_id, ros::Time stamp);

geometry_msgs::Point transformPoint(geometry_msgs::Point input, string target_frame, string source_frame, ros::Time stamp);

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

  // get a binary image
  cv::Mat output;
  detectStrap(cv_ptr, output);

  PointCloudXYZRGB::Ptr cloud_filtered = filterCloud(output, cloud, cloud_msg->header.frame_id, cloud_msg->header.stamp);

  cloud_filtered->header.stamp = ros::Time::now();
  obsCloudPub.publish(cloud_filtered);
}

void detectStrap(cv_bridge::CvImagePtr cv_ptr, cv::Mat &output)
{
  std::cout << "In detectStrap" << std::endl;
  try
  {
    cv::cvtColor(cv_ptr->image, output, CV_BGR2HSV);
    
    cv::Mat temp;

    // Make a vector of Mats to hold the individual B,G,R channels
    std::vector<cv::Mat> mats;

    // Split the input into 3 separate channels
    split(temp,mats);

    // create the range of HSV values that determine the color we desire to threshold based on launch file params
    cv::Scalar lowerBound = cv::Scalar(hl,sl,vl);
    cv::Scalar upperBound = cv::Scalar(hh,sh,vh);

    //threshold the image based on the HSV values
    cv::inRange(output,lowerBound,upperBound,output);

    cv::erode(output, output, cv::Mat());
    cv::dilate(output, output, cv::Mat(), cv::Point(-1,-1), dilationIterations);

    // display the image
    std::cout << "About to display image" << std::endl;
    cv::imshow("binary",output);
    cvWaitKey(5);
  }
  catch(cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert to 'bgr8'. Ex was %s", e.what());
  }
}


PointCloudXYZRGB::Ptr filterCloud(cv::Mat &input, PointCloudXYZRGB &cloud, string frame_id, ros::Time stamp)
{
  PointCloudXYZRGB::Ptr obsCloudMsg (new PointCloudXYZRGB);
  obsCloudMsg->header.frame_id = robot_frame;
  obsCloudMsg->height = 480;
  obsCloudMsg->width = 640;
  for(int row=0; row<480; row++)
  {
    for(int col=0; col<640; col++)
    {
      // if it matches the color
      if(input.at<cv::Vec3b>(row,col)[0] > 0)
      {
	pcl::PointXYZRGB pcl_pt = cloud.at(col,row);

	pcl_pt = transformPoint(pcl_pt, robot_frame, frame_id, stamp);
	obsCloudMsg->points.push_back(pcl_pt);
      }
      else
      {
	pcl::PointXYZRGB pcl_pt = cloud.at(col,row);

	// check for NaN values
	if(pcl_pt.x != pcl_pt.x || pcl_pt.y != pcl_pt.y || pcl_pt.z != pcl_pt.z)
	  continue;

	pcl_pt = transformPoint(pcl_pt, robot_frame, frame_id, stamp);

	// only add it if the point is within the tolerances
	if(pcl_pt.z <= zHigh && pcl_pt.z >= zLow)
	  obsCloudMsg->points.push_back(pcl_pt);
      }	
    }
  }	
  
  return obsCloudMsg;
}

pcl::PointXYZRGB transformPoint(pcl::PointXYZRGB pcl_pt, string target_frame, string cloud_frame_id, ros::Time stamp)
{
  geometry_msgs::Point geom_pt;

  geom_pt.x = pcl_pt.x; 
  geom_pt.y = pcl_pt.y; 
  geom_pt.z = pcl_pt.z;
  
  geom_pt = transformPoint(geom_pt,target_frame, cloud_frame_id, stamp);

  pcl::PointXYZRGB result;
  result.x = geom_pt.x;
  result.y = geom_pt.y;
  result.z = geom_pt.z;

  return result;
}

geometry_msgs::Point transformPoint(geometry_msgs::Point input, string target_frame, string source_frame, ros::Time stamp)
{
  geometry_msgs::Point geom_pt;

  // the TF package requires inputs to be in the form Stamped<sometype>
  tf::Stamped<tf::Point> geom_pt_tf, temp_pt_tf;
  pointMsgToTF(input, geom_pt_tf);
  geom_pt_tf.frame_id_ = source_frame;
  geom_pt_tf.stamp_    = stamp;
	
  try 
  {
    tfl->transformPoint(target_frame, geom_pt_tf, temp_pt_tf);
  }
  catch(tf::TransformException& ex) 
  {
    ROS_ERROR_STREAM(boost::format("Failed to transform point pose from \"%s\" to \"%s\" frame: %s")
		                   %geom_pt_tf.frame_id_ %target_frame %ex.what());
    geometry_msgs::Point empty_pt;
    return empty_pt;
  }
  tf::pointTFToMsg(temp_pt_tf, geom_pt);
	
  return geom_pt;
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
  private_nh.param("hl", hl, 0);
  private_nh.param("hh", hh, 255);
  private_nh.param("sl", sl, 0);
  private_nh.param("sh", sh, 255);
  private_nh.param("vl", vl, 0);
  private_nh.param("vh", vh, 255);
  private_nh.param("dilationIterations", dilationIterations, 5);
  private_nh.param("zLow", zLow, 0.1);
  private_nh.param("zHigh", zHigh, 1000.0);
  private_nh.param("global_frame", global_frame, string("map"));
  private_nh.param("robot_frame", robot_frame, string("base_link"));

  std::cout << "hl: " << hl << std::endl;
  std::cout << "hh: " << hh << std::endl;
  std::cout << "sl: " << sl << std::endl;
  std::cout << "sh: " << sh << std::endl;
  std::cout << "vl: " << vl << std::endl;
  std::cout << "vh: " << vh << std::endl;
  std::cout << "dilationIterations: " << dilationIterations << std::endl;
  std::cout << "zLow: " << zLow << std::endl;
  std::cout << "zHigh: " << zHigh << std::endl;
  std::cout << "global_frame: " << global_frame << std::endl;
  std::cout << "robot_frame: " << robot_frame << std::endl;

  // naptime
  ros::Rate naptime(20); // will sleep to enforce a rate of 20 Hz
  
  // Point Cloud publisher
  obsCloudPub = nh.advertise<sensor_msgs::PointCloud2> ("obsCloud",1);

  // Subscribe to an image, cloud, and camera info.
  // Note the use of image_transport::SubscriberFilter and message_filters::Subscriber.  These allow for synchronization of the topics.
  image_transport::SubscriberFilter                    image_sub   (it, "camera/rgb/image_rect_color", 1);
  message_filters::Subscriber<sensor_msgs::PointCloud2>        cloud_sub   (nh, "camera/depth_registered/points", 1);
  message_filters::Subscriber<sensor_msgs::CameraInfo> cam_info_sub(nh, "camera/rgb/camera_info", 1);
	
  // This sync policy will invoke a callback if it receives one of each message with matching timestamps
  typedef message_filters::sync_policies::ApproximateTime <sensor_msgs::Image, sensor_msgs::PointCloud2, sensor_msgs::CameraInfo> MySyncPolicy;
	
  // Synchronize the three topics.  MySyncPolicy(10) tells it to maintain a buffer of 10 messages.
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_sub, cloud_sub, cam_info_sub);
	
  // Hook the callback into the sync policy
  sync.registerCallback( boost::bind(&allCB, _1, _2, _3) );


  ROS_INFO("Done initializing, going into spin mode.");

  while(ros::ok())
  {
    ros::spinOnce();
    naptime.sleep();
  }
  
  return 0;
}
