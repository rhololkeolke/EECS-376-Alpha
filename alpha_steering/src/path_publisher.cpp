#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <alpha_steering/PathSegment.h>
#include <alpha_steering/SegStatus.h>
#include <iostream>
#include <fstream>
#include <tf/transform_datatypes.h>

const double PI=3.14159;

using namespace std;

bool segComplete = true;
int seg_number = 0;

int numSegs = 5;

void segStatusCallback(const alpha_steering::SegStatus::ConstPtr& status)
{
  //seg_number = status->seg_number;
  // segComplete is only true once and we only care about when segComplete transitions to true
  // so only update if the segment is currently not completed
  if(!segComplete)
    segComplete = status->segComplete;
}

int main(int argc, char **argv)
{
	string base = argv[0];
	int path_loc = base.find("bin/");
	string new_part = "config/waypoints.txt";
	base.replace(path_loc,14,new_part);
	cout << "base: " << base << endl;
	
	std::ifstream infile(base.c_str());
	//infile.open("../config/alpha_steering_constants.txt");
  ros::init(argc,argv,"path_publisher");
  ros::NodeHandle n;
  
  ros::Publisher pathPub = n.advertise<alpha_steering::PathSegment>("path_seg",1);
  ros::Subscriber segSub = n.subscribe<alpha_steering::SegStatus>("seg_status",1,segStatusCallback);

  ros::Rate naptime(10);

  while(!ros::Time::isValid()) {}

  alpha_steering::PathSegment currSeg;
  
  while(ros::ok() && seg_number < numSegs)
  {
    ros::spinOnce();
    if(segComplete == true)
    {
      seg_number++;
      segComplete = false;

      switch(seg_number)
      {
      case 1:
	ROS_INFO("Sending out path %i",seg_number);
	if(infile.good()){
		infile >> currSeg.seg_length;
		infile >> currSeg.ref_point.x;
		infile >> currSeg.ref_point.y;
		infile >> currSeg.init_tan_angle;
		currSeg.init_tan_angle = tf::createQuaternionMsgFromYaw(currSeg.init_tan_angle*PI/180.0);
	} else {
		cout << "there was a problem reading the file for input" << endl;
	}
	currSeg.seg_number = 1;
	currSeg.seg_type = 1;
	
	break;
      case 2:
	ROS_INFO("Sending out path %i",seg_number);
	currSeg.seg_number = 2;
	currSeg.seg_type = 3;
	if(infile.good()){
		infile >> currSeg.seg_length;
		infile >> currSeg.ref_point.x;
		infile >> currSeg.ref_point.y;
		infile >> currSeg.init_tan_angle;
		currSeg.init_tan_angle = tf::createQuaternionMsgFromYaw(currSeg.init_tan_angle*PI/180.0);
	} else {
		cout << "there was a problem reading the file for input" << endl;
	}
	
	break;
      case 3:
	ROS_INFO("Sending out path %i",seg_number);
	currSeg.seg_number = 3;
	currSeg.seg_type = 1;
	if(infile.good()){
		infile >> currSeg.seg_length;
		infile >> currSeg.ref_point.x;
		infile >> currSeg.ref_point.y;
		infile >> currSeg.init_tan_angle;
		currSeg.init_tan_angle = tf::createQuaternionMsgFromYaw(currSeg.init_tan_angle*PI/180.0);
	} else {
		cout << "there was a problem reading the file for input" << endl;
	}
    break;
      case 4:
	ROS_INFO("Sending out path %i",seg_number);
	currSeg.seg_number = 4;
	currSeg.seg_type = 3;
	if(infile.good()){
		infile >> currSeg.seg_length;
		infile >> currSeg.ref_point.x;
		infile >> currSeg.ref_point.y;
		infile >> currSeg.init_tan_angle;
		currSeg.init_tan_angle = tf::createQuaternionMsgFromYaw(currSeg.init_tan_angle*PI/180.0);
	} else {
		cout << "there was a problem reading the file for input" << endl;
	}
	break;
      case 5:
	ROS_INFO("Sending out path %i",seg_number);
	currSeg.seg_number = 5;
	currSeg.seg_type = 1;
	if(infile.good()){
		infile >> currSeg.seg_length;
		infile >> currSeg.ref_point.x;
		infile >> currSeg.ref_point.y;
		infile >> currSeg.init_tan_angle;
		currSeg.init_tan_angle = tf::createQuaternionMsgFromYaw(currSeg.init_tan_angle*PI/180.0);
	} else {
		cout << "there was a problem reading the file for input" << endl;
	}
	break;
      default:
	ROS_INFO("I don't know what to publish, seg_number=%i",seg_number);
      }  
    }
    //ROS_INFO("Publishing!");
    pathPub.publish(currSeg);
    naptime.sleep();
  }
	infile.close();
  return 0;
}
