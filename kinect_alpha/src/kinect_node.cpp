#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/cv.h>
#include <opencv2/highgui.h>
#include <cv_bridge/CvBridge.h>
#include <geometry_msgs/Twist.h>

#include <kinect_alpha/lib_blob.h>

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
  cv::Mat image;
  cv::Mat output;
  try
  {
    image = cv::Mat(bridge.imgMsgToCv(msg, "bgr8"));

  }
  catch (sensor_msgs::CvBridgeException& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'. E was %s", msg->encoding.c_str(), e.what());
  }
  try {
    //normalizeColors(image, output);
    //blobfind(image, output);
    findLines(image, output);
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
