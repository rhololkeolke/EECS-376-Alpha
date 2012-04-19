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

// Boost Libraries
#include <boost/format.hpp>

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

// Shorthand for our point cloud type
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;

// Global variables here
ros::Publisher             cloud_pub_;
image_transport::Publisher image_pub_;
string window_name_;
tf::TransformListener* tfl_;
string global_frame_ = "map";
string robot_frame_ = "base_link";

// parameters from launch file
int hl, hh, sl, sh, vl, vh;
int dilationIterations;
double zTolLow, zTolHigh;
int numBins;

bool goal_pt_exists = false;
geometry_msgs::Point goal_pt;


// function that calls all of the helper functions
geometry_msgs::Point findClosestCentroid(PointCloudXYZRGB &cloud, cv_bridge::CvImagePtr cv_ptr, string cloud_frame_id, ros::Time stamp);

// makes a binary image with white wherever there is the strap color
void detectStrap(cv_bridge::CvImagePtr cv_ptr, cv::Mat &output);

// puts each point in the correct bin if it is white and within the z tolerances
void putInBins(PointCloudXYZRGB &cloud, cv::Mat &input, std::vector<std::vector<geometry_msgs::Point> > &bins, string cloud_frame_id, ros::Time stamp);

// helper function for putInBins
geometry_msgs::Point transformPoint(pcl::PointXYZRGB pcl_pt, string target_frame, string cloud_frame_id, ros::Time stamp);

geometry_msgs::Point transformPoint(geometry_msgs::Point input, string target_frame, string cloud_frame_id, ros::Time stamp);


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

	
	goal_pt = findClosestCentroid(cloud, cv_ptr, cloud_msg->header.frame_id, cloud_msg->header.stamp);

	cv::imshow(window_name_.c_str(), cv_ptr->image);
	cvWaitKey(5);
	
	// Publish the modified image
	//image_pub_.publish(cv_ptr->toImageMsg());
}

geometry_msgs::Point findClosestCentroid(PointCloudXYZRGB &cloud, cv_bridge::CvImagePtr cv_ptr, string cloud_frame_id, ros::Time stamp)
{
  // get a binary image
  cv::Mat output;
  detectStrap(cv_ptr,output);
  
  // create the bins vector which will store all of the classified points
  std::vector<std::vector<geometry_msgs::Point> > bins;

  for(int rowBin=0; rowBin < numBins; rowBin++)
  {
    for(int colBin=0; colBin < numBins; colBin++)
    {
      std::vector<geometry_msgs::Point> bin;
      bins.push_back(bin);
    }
  }

  // classify all of the wanted points into bins
  putInBins(cloud,output,bins,cloud_frame_id,stamp);

  // go through each bin and compute the centroid.  If the centroid is closer than the last
  // set that as the new closest point
  geometry_msgs::Point best_point;
  double closestDist = 100.0;
  while(!bins.empty())
  {
    std::vector<geometry_msgs::Point> bin = bins.back();
    bins.pop_back(); // after this iteration we never care about this bin again
    double centroidX = 0.0;
    double centroidY = 0.0;
    if(!bin.empty())
    {
      goal_pt_exists = true;
      for(int j=0; j<bin.size(); j++)
      {
	centroidX += bin[j].x;
	centroidY += bin[j].y;
      }
      centroidX = centroidX/(double)bin.size();
      centroidY = centroidY/(double)bin.size();
      
      // Check the distance to the base link
      double distance = sqrt(pow(centroidX,2) + pow(centroidY,2));

      // Check if this centroid is closer
      if(distance < closestDist)
      {
	closestDist = distance;
	best_point.x = centroidX;
	best_point.y = centroidY;
      }
      //std::cout << "Closest Centroid:\n\tx: " << best_point.x << "\n\ty: " << best_point.y << std::endl;
    }
    /* else
    {
      std::cout << "No pixels in bin" << std::endl;
      }*/
  }
  
  std::cout << "Best point in base_link: " << best_point << std::endl;
  //best_point = transformPoint(best_point, global_frame_, "base_link", stamp);
  //std::cout << "Best point in map: " << best_point << std::endl;

  return best_point;
}

void detectStrap(cv_bridge::CvImagePtr cv_ptr, cv::Mat &output)
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
    cv::Scalar lowerBound = cv::Scalar(hl,sl,vl);
    cv::Scalar upperBound = cv::Scalar(hh,sh,vh);

    //threshold the image based on the HSV values
    cv::inRange(output,lowerBound,upperBound,output);

    cv::erode(output, output, cv::Mat());
    cv::dilate(output, output, cv::Mat(), cv::Point(-1,-1), dilationIterations);

    // display the image
    cv::imshow("binary",output);
    cvWaitKey(5);
  }
  catch(cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert to 'bgr8'. Ex was %s", e.what());
  }
}

void putInBins(PointCloudXYZRGB &cloud, cv::Mat &input, std::vector<std::vector<geometry_msgs::Point> > &bins, string cloud_frame_id, ros::Time stamp)
{

  //std::cout << "Running putInBins" << std::endl;
  //std::cout << "numBins: " << numBins << std::endl;
  //std::cout << "bins.size() " << bins.size() << std::endl;
  int colStep = floor(640/numBins);
  int rowStep = floor(480/numBins);

  for(int row=0; row<480; row++)
  {
    for(int col=0; col<640; col++)
    {  
      if(input.at<cv::Vec3b>(row,col)[0] > 0)
      {

	pcl::PointXYZRGB pcl_pt = cloud.at(col, row);
	geometry_msgs::Point geom_pt = transformPoint(pcl_pt, robot_frame_, cloud_frame_id, stamp);
	
	if(geom_pt.z < zTolHigh && geom_pt.z > zTolLow)
	{
	  // figure out which vector bin in the bins vector the point should go in
	  // might want to make the bins based on x,y map space and not i,j camera space
	  // but for the first draft proof of concept it should suffice
	  int index = floor(row/rowStep)*numBins + floor(col/colStep);

	  // actually put the point in that bin
	  if(index > bins.size())
	  {
	    ROS_ERROR("Index larger than number of bins");
	    //std::cout << "index: " << index << std::endl;
	  }
	  else
	  {
	    bins[index].push_back(geom_pt);
	    //std::cout << "Found a valid point!" << std::endl;
	    //std::cout << "Value at " << row << ", " << col << " is " << (int)input.at<cv::Vec3b>(row,col)[0] << std::endl;
	    //std::cout << "Point Location in base_link:\n\tx: " << geom_pt.x << "\n\ty: " << geom_pt.y << "\n\tz: " << geom_pt.z << std::endl;
	    //std::cout << "index: " << index << std::endl;
	  }
	}
      }
    }
  }
}

geometry_msgs::Point transformPoint(pcl::PointXYZRGB pcl_pt, string target_frame, string cloud_frame_id, ros::Time stamp)
{
  geometry_msgs::Point geom_pt;

  geom_pt.x = pcl_pt.x; 
  geom_pt.y = pcl_pt.y; 
  geom_pt.z = pcl_pt.z;
	
  // the TF package requires inputs to be in the form Stamped<sometype>
  tf::Stamped<tf::Point> geom_pt_tf, temp_pt_tf;
  pointMsgToTF(geom_pt, geom_pt_tf);
  geom_pt_tf.frame_id_ = cloud_frame_id;
  geom_pt_tf.stamp_    = stamp;
	
  try 
  {
    tfl_->transformPoint(target_frame, geom_pt_tf, temp_pt_tf);
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

geometry_msgs::Point transformPoint(geometry_msgs::Point input, string target_frame, string cloud_frame_id, ros::Time stamp)
{
  geometry_msgs::Point geom_pt;
 // the TF package requires inputs to be in the form Stamped<sometype>
  tf::Stamped<tf::Point> geom_pt_tf, temp_pt_tf;
  pointMsgToTF(geom_pt, geom_pt_tf);
  geom_pt_tf.frame_id_ = cloud_frame_id;
  geom_pt_tf.stamp_    = stamp;
	
  try 
  {
    tfl_->transformPoint(target_frame, geom_pt_tf, temp_pt_tf);
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
	private_nh.param("hl", hl, 0);
	private_nh.param("hh", hh, 255);
	private_nh.param("sl", sl, 0);
	private_nh.param("sh", sh, 255);
	private_nh.param("vl", vl, 0);
	private_nh.param("vh", vh, 255);
	private_nh.param("dilationIterations", dilationIterations, 5);
	private_nh.param("zTolLow", zTolLow, -0.5);
	private_nh.param("zTolHigh", zTolHigh, 0.5);
	private_nh.param("numBins", numBins, 5);

	private_nh.param("global_frame", global_frame_, string("map"));
	private_nh.param("robot_frame", robot_frame_, string("base_link"));
	
	std::cout << "hl: " << hl << std::endl;
	std::cout << "hh: " << hh << std::endl;
	std::cout << "sl: " << sl << std::endl;
	std::cout << "sh: " << sh << std::endl;
	std::cout << "vl: " << vl << std::endl;
	std::cout << "vh: " << vh << std::endl;
	std::cout << "dilationIterations: " << dilationIterations << std::endl;
	std::cout << "zTolLow: " << zTolLow << std::endl;
	std::cout << "zTolHigh: " << zTolHigh << std::endl;
	std::cout << "numBins: " << numBins << std::endl;
	std::cout << "global_frame_: " << global_frame_ << std::endl;
	std::cout << "robot_frame_: " << robot_frame_ << std::endl;
	
	// naptime
	ros::Rate naptime(20);// will sleep to enforce a rate of 20 Hz

	// Centroid Point Publisher
	ros::Publisher centroidPub = nh.advertise<msg_alpha::CentroidPoints>("centroid_point",1);
	
	// Subscribe to an image, cloud, and camera info.
	// Note the use of image_transport::SubscriberFilter and message_filters::Subscriber.  These allow for synchronization of the topics.
	image_transport::SubscriberFilter                    image_sub   (it, "camera/rgb/image_rect_color", 1);
	message_filters::Subscriber<sensor_msgs::PointCloud2>        cloud_sub   (nh, "camera/depth_registered/points", 1);
	message_filters::Subscriber<sensor_msgs::CameraInfo> cam_info_sub(nh, "camera/rgb/camera_info", 1);
	
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
	  msg_alpha::CentroidPoints pointsMsg;
	  pointsMsg.exists = goal_pt_exists;
	  pointsMsg.point = goal_pt;
	  centroidPub.publish(pointsMsg);
	  naptime.sleep();
	  
	}
	return 0;
}
