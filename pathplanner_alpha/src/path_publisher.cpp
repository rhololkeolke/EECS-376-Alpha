#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <msg_alpha/PathSegment.h>
#include <msg_alpha/SegStatus.h>
#include <iostream>
#include <tf/transform_datatypes.h>
#include <stack>

const double PI=3.14159;

using namespace std;

bool segComplete = true;
int seg_number = 0;

int numSegs = 5;


//Stack is created
stack <msg_alpha::PathSegment> pathStack;

void segStatusCallback(const msg_alpha::SegStatus::ConstPtr& status)
{
  //seg_number = status->seg_number;
  // segComplete is only true once and we only care about when segComplete transitions to true
  // so only update if the segment is currently not completed
  if(!segComplete)
    segComplete = status->segComplete;
}

void initStack(){
	
	msg_alpha::PathSegment Seg;		

	Seg.seg_number = 5;
	Seg.seg_type = 1;
	Seg.seg_length = 1;
	Seg.ref_point.x = -3.28;
	Seg.ref_point.y = 20.8;
	Seg.init_tan_angle = tf::createQuaternionMsgFromYaw(45.22*PI/180.0);
	pathStack.push(Seg);

	Seg.seg_number = 5;
	Seg.seg_type = 1;
	Seg.seg_length = 1;
	Seg.ref_point.x = -3.28;
	Seg.ref_point.y = 20.8;
	Seg.init_tan_angle = tf::createQuaternionMsgFromYaw(45.22*PI/180.0);
	pathStack.push(Seg);

	Seg.seg_number = 4;
	Seg.seg_type = 3;
	Seg.seg_length = -PI/2;
	Seg.ref_point.x = 14.84;
	Seg.ref_point.y = 3.91;
	pathStack.push(Seg);

	ROS_INFO("Sending out path %i",seg_number);
	Seg.seg_number = 3;
	Seg.seg_type = 1;
	Seg.seg_length = 12.34;
	Seg.ref_point.x = 5.45;
	Seg.ref_point.y = 11.92;
	Seg.init_tan_angle = tf::createQuaternionMsgFromYaw(136.0*PI/180.0);
	pathStack.push(Seg);

	ROS_INFO("Sending out path %i",seg_number);
	Seg.seg_number = 2;
	Seg.seg_type = 3;
	Seg.seg_length = -PI/2;
	Seg.ref_point.x = 5.23;
	Seg.ref_point.y = 11.92;		
	pathStack.push(Seg);

	ROS_INFO("Sending out path %i",seg_number);
	Seg.seg_number = 1;
	Seg.seg_type = 1;
	Seg.seg_length = 4.2;
	Seg.ref_point.x = 8.27;
	Seg.ref_point.y = 14.74;
	Seg.init_tan_angle = tf::createQuaternionMsgFromYaw(-135.7*PI/180.0);
	pathStack.push(Seg);
}

void detour()
{

	//this method assumes that the lidar node will wait three seconds
	//before publishing a segStatus of !OK

	ros::Subscriber 


}
 
int main(int argc, char **argv)
{
  	ros::init(argc,argv,"path_publisher");
  	ros::NodeHandle n;

	ros::Subscriber 
	msg_alpha::PathSegment Seg;



 	ros::Publisher pathPub = n.advertise<msg_alpha::PathSegment>("path_seg",1);
  	ros::Subscriber segSub = n.subscribe<msg_alpha::SegStatus>("seg_status",1,segStatusCallback);
	//ros::Subscriber obstacles 

  	ros::Rate naptime(10);

  	while(!ros::Time::isValid()) {}	
  
  	while(ros::ok() && !pathStack.empty())
  	{
  		//While seg status is ok.
		if(SEG STATUS == OK)
		{
	    	ros::spinOnce();
			if(segComplete == true)
		    {
		    	currSeg = pathStack.top();
		    	pathStack.pop();
		    	segComplete = false;
				ROS_INFO("I published another node! Be proud...");
		    }  
	
		}else(SEG STATUS == !OK){
			detour();
		}


		//ROS_INFO("Publishing!");
		pathPub.publish(currSeg);
		naptime.sleep();
	}
  return 0;
}
