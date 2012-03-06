#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <steering/PathSegment.h>
#include <iostream>
#include <tf/transform_datatypes.h>
#include <steering/SegStatus.h>

const double PI=3.14159;

bool segComplete = false;
int seg_number = 0;

void segStatusCallback(const steering::SegStatus::ConstPtr& status)
{
  if(!segComplete)
    segComplete = status->segComplete;

  seg_number = status->seg_number;
}

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc,argv,"path_publisher");
  ros::NodeHandle n;
  
  ros::Publisher pathPub = n.advertise<steering::PathSegment>("path_seg",1);
  ros::Subscriber segStatusSub = n.subscribe<steering::SegStatus>("seg_status",10, segStatusCallback);

  ros::AsyncSpinner spinner(0);
  spinner.start();

  ros::Rate naptime(20);

  while(!ros::Time::isValid()) {}

  steering::PathSegment currSeg;

  int i = 1;
  while(ros::ok())
  {
    if(segComplete)
    {
      
      i = i + 1;
      ROS_INFO("New i value: %i", i);
      segComplete = false;
    }
    switch(i)
      {
      case 1:
	ROS_INFO("Case 1");
  currSeg.seg_number = 1;
  currSeg.seg_type = 1;
  currSeg.seg_length = 4.2;
  currSeg.ref_point.x = 8.27;
  currSeg.ref_point.y = 14.74;
  currSeg.init_tan_angle = tf::createQuaternionMsgFromYaw(-137.16*PI/180.0);
  break;
      case 2:
	ROS_INFO("Case 2");
  currSeg.seg_number = 2;
  currSeg.seg_type = 3;
  currSeg.seg_length = -PI/2;
  currSeg.ref_point.x = 5.23;
  currSeg.ref_point.y = 11.92;
  break;
      case 3:  
	ROS_INFO("Case 3");
  currSeg.seg_number = 3;
  currSeg.seg_type = 1;
  currSeg.seg_length = 12.34;
  currSeg.ref_point.x = 5.45;
  currSeg.ref_point.y = 11.92;
  currSeg.init_tan_angle = tf::createQuaternionMsgFromYaw(129.0*PI/180.0);
  break;
      case 4:
	ROS_INFO("Case 4");
  currSeg.seg_number = 4;
  currSeg.seg_type = 3;
  currSeg.seg_length = -PI/2;
  currSeg.ref_point.x = 14.84;
  currSeg.ref_point.y = 3.91;
  break;
      case 5:
	ROS_INFO("Case 5");
  currSeg.seg_number = 5;
  currSeg.seg_type = 1;
  currSeg.seg_length = 2;
  currSeg.ref_point.x = -3.28;
  currSeg.ref_point.y = 20.8;
  currSeg.init_tan_angle = tf::createQuaternionMsgFromYaw(45.22*PI/180.0);
  break;
      default:
	ROS_INFO("Default");
  currSeg.seg_number = -1;
  currSeg.seg_type = 1;
  currSeg.seg_length = 0.0;
  currSeg.ref_point.y = 0.0;
  currSeg.ref_point.x = 0.0;
  currSeg.init_tan_angle = tf::createQuaternionMsgFromYaw(0.0);
      }

  pathPub.publish(currSeg);
  
  naptime.sleep();
  }
  return 0;
}
