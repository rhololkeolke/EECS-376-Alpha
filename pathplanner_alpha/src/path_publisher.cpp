#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <msg_alpha/PathSegment.h>
#include <msg_alpha/SegStatus.h>
#include <iostream>
#include <tf/transform_datatypes.h>

const double PI=3.14159;

using namespace std;

bool segComplete = true;
int seg_number = 0;

int numSegs = 5;

void segStatusCallback(const msg_alpha::SegStatus::ConstPtr& status)
{
  //seg_number = status->seg_number;
  // segComplete is only true once and we only care about when segComplete transitions to true
  // so only update if the segment is currently not completed
  if(!segComplete)
    segComplete = status->segComplete;
}

int main(int argc, char **argv)
{
  ros::init(argc,argv,"path_publisher");
  ros::NodeHandle n;
  
  ros::Publisher pathPub = n.advertise<msg_alpha::PathSegment>("path_seg",1);
  ros::Subscriber segSub = n.subscribe<msg_alpha::SegStatus>("seg_status",1,segStatusCallback);
  ros::Subscriber 

  ros::Rate naptime(10);

  while(!ros::Time::isValid()) {}

  msg_alpha::PathSegment currSeg;
  
  while(ros::ok() && seg_number < numSegs)
  {
  	while()
    ros::spinOnce();
    if(segComplete == true)
    {
      seg_number++;
      segComplete = false;

      switch(seg_number)
      {
      case 1:
	ROS_INFO("Sending out path %i",seg_number);
	currSeg.seg_number = 1;
	currSeg.seg_type = 1;
	currSeg.seg_length = 4.2;
	currSeg.ref_point.x = 8.27;
	currSeg.ref_point.y = 14.74;
	currSeg.init_tan_angle = tf::createQuaternionMsgFromYaw(-135.7*PI/180.0);
	
	break;
      case 2:
	ROS_INFO("Sending out path %i",seg_number);
	currSeg.seg_number = 2;
	currSeg.seg_type = 3;
	currSeg.seg_length = -PI/2;
	currSeg.ref_point.x = 5.23;
	currSeg.ref_point.y = 11.92;
	
	break;
      case 3:
	ROS_INFO("Sending out path %i",seg_number);
	currSeg.seg_number = 3;
	currSeg.seg_type = 1;
	currSeg.seg_length = 12.34;
	currSeg.ref_point.x = 5.45;
	currSeg.ref_point.y = 11.92;
	currSeg.init_tan_angle = tf::createQuaternionMsgFromYaw(136.0*PI/180.0);
    break;
      case 4:
	ROS_INFO("Sending out path %i",seg_number);
	currSeg.seg_number = 4;
	currSeg.seg_type = 3;
	currSeg.seg_length = -PI/2;
	currSeg.ref_point.x = 14.84;
	currSeg.ref_point.y = 3.91;
	break;
      case 5:
	ROS_INFO("Sending out path %i",seg_number);
	currSeg.seg_number = 5;
	currSeg.seg_type = 1;
	currSeg.seg_length = 2;
	currSeg.ref_point.x = -3.28;
	currSeg.ref_point.y = 20.8;
	currSeg.init_tan_angle = tf::createQuaternionMsgFromYaw(45.22*PI/180.0);
	break;
      default:
	ROS_INFO("I don't know what to publish, seg_number=%i",seg_number);
      }  
    }
    //ROS_INFO("Publishing!");
    pathPub.publish(currSeg);
    naptime.sleep();
  }
  return 0;
}
