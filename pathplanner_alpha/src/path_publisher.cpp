#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <msg_alpha/PathSegment.h>
#include <msg_alpha/SegStatus.h>
#include <msg_alpha/Obstacles.h>
#include <iostream>
#include <tf/transform_datatypes.h>
#include <stack>

const double PI=3.14159;

using namespace std;

bool segComplete = true;
bool abort = false;
int seg_number = 0;
msg_alpha::Obstacles lastObs;
int numSegs = 5;
msg_alpha::PathSegment currSeg;
//Stack is created
stack <msg_alpha::PathSegment> pathStack;
ros::Publisher pathPub;

void segStatusCallback(const msg_alpha::SegStatus::ConstPtr& status)
{
	//seg_number = status->seg_number;
	// segComplete is only true once and we only care about when segComplete transitions to true
	// so only update if the segment is currently not completed
	if(!segComplete)
	{
		segComplete = status->segComplete;
	}
	abort = status->abort;
}

void obstaclesCallback(const msg_alpha::Obstacles::ConstPtr& obstacles)
{
	if(obstacles.exists)
	{
		lastObs = obstacles;
	}
}

void initStack()
{

	msg_alpha::PathSegment Seg;		

	//Initial segment
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

	//First turn
	Seg.seg_number = 4;
	Seg.seg_type = 3;
	Seg.seg_length = -PI/2;
	Seg.ref_point.x = 14.84;
	Seg.ref_point.y = 3.91;
	pathStack.push(Seg);

	//Long straight away from the lab
	Seg.seg_number = 3;
	Seg.seg_type = 1;
	Seg.seg_length = 1;
	Seg.ref_point.x = 5.45;
	Seg.ref_point.y = 11.92;
	Seg.init_tan_angle = tf::createQuaternionMsgFromYaw(136.0*PI/180.0);
	pathStack.push(Seg);

	Seg.seg_number = 3;
	Seg.seg_type = 1;
	Seg.seg_length = 1;
	Seg.ref_point.x = 5.45;
	Seg.ref_point.y = 11.92;
	Seg.init_tan_angle = tf::createQuaternionMsgFromYaw(136.0*PI/180.0);
	pathStack.push(Seg);

	Seg.seg_number = 3;
	Seg.seg_type = 1;
	Seg.seg_length = 1;
	Seg.ref_point.x = 5.45;
	Seg.ref_point.y = 11.92;
	Seg.init_tan_angle = tf::createQuaternionMsgFromYaw(136.0*PI/180.0);
	pathStack.push(Seg);

	Seg.seg_number = 3;
	Seg.seg_type = 1;
	Seg.seg_length = 1;
	Seg.ref_point.x = 5.45;
	Seg.ref_point.y = 11.92;
	Seg.init_tan_angle = tf::createQuaternionMsgFromYaw(136.0*PI/180.0);
	pathStack.push(Seg);

	Seg.seg_number = 3;
	Seg.seg_type = 1;
	Seg.seg_length = 1;
	Seg.ref_point.x = 5.45;
	Seg.ref_point.y = 11.92;
	Seg.init_tan_angle = tf::createQuaternionMsgFromYaw(136.0*PI/180.0);
	pathStack.push(Seg);

	Seg.seg_number = 3;
	Seg.seg_type = 1;
	Seg.seg_length = 1;
	Seg.ref_point.x = 5.45;
	Seg.ref_point.y = 11.92;
	Seg.init_tan_angle = tf::createQuaternionMsgFromYaw(136.0*PI/180.0);
	pathStack.push(Seg);

	Seg.seg_number = 3;
	Seg.seg_type = 1;
	Seg.seg_length = 1;
	Seg.ref_point.x = 5.45;
	Seg.ref_point.y = 11.92;
	Seg.init_tan_angle = tf::createQuaternionMsgFromYaw(136.0*PI/180.0);
	pathStack.push(Seg);

	Seg.seg_number = 3;
	Seg.seg_type = 1;
	Seg.seg_length = 1;
	Seg.ref_point.x = 5.45;
	Seg.ref_point.y = 11.92;
	Seg.init_tan_angle = tf::createQuaternionMsgFromYaw(136.0*PI/180.0);
	pathStack.push(Seg);

	Seg.seg_number = 3;
	Seg.seg_type = 1;
	Seg.seg_length = 1;
	Seg.ref_point.x = 5.45;
	Seg.ref_point.y = 11.92;
	Seg.init_tan_angle = tf::createQuaternionMsgFromYaw(136.0*PI/180.0);
	pathStack.push(Seg);

	Seg.seg_number = 3;
	Seg.seg_type = 1;
	Seg.seg_length = 1;
	Seg.ref_point.x = 5.45;
	Seg.ref_point.y = 11.92;
	Seg.init_tan_angle = tf::createQuaternionMsgFromYaw(136.0*PI/180.0);
	pathStack.push(Seg);

	Seg.seg_number = 3;
	Seg.seg_type = 1;
	Seg.seg_length = 1;
	Seg.ref_point.x = 5.45;
	Seg.ref_point.y = 11.92;
	Seg.init_tan_angle = tf::createQuaternionMsgFromYaw(136.0*PI/180.0);
	pathStack.push(Seg);

	Seg.seg_number = 3;
	Seg.seg_type = 1;
	Seg.seg_length = 1;
	Seg.ref_point.x = 5.45;
	Seg.ref_point.y = 11.92;
	Seg.init_tan_angle = tf::createQuaternionMsgFromYaw(136.0*PI/180.0);
	pathStack.push(Seg);

	Seg.seg_number = 3;
	Seg.seg_type = 1;
	Seg.seg_length = 1;
	Seg.ref_point.x = 5.45;
	Seg.ref_point.y = 11.92;
	Seg.init_tan_angle = tf::createQuaternionMsgFromYaw(136.0*PI/180.0);
	pathStack.push(Seg);

	Seg.seg_number = 3;
	Seg.seg_type = 1;
	Seg.seg_length = .34;
	Seg.ref_point.x = 5.45;
	Seg.ref_point.y = 11.92;
	Seg.init_tan_angle = tf::createQuaternionMsgFromYaw(136.0*PI/180.0);
	pathStack.push(Seg);

	//Turn toward the coffee machines
	Seg.seg_number = 2;
	Seg.seg_type = 3;
	Seg.seg_length = -PI/2;
	Seg.ref_point.x = 5.23;
	Seg.ref_point.y = 11.92;		
	pathStack.push(Seg);

	//Straight to the coffee machines
	Seg.seg_number = 1;
	Seg.seg_type = 1;
	Seg.seg_length = 4.2;
	Seg.ref_point.x = 8.27;
	Seg.ref_point.y = 14.74;
	Seg.init_tan_angle = tf::createQuaternionMsgFromYaw(-135.7*PI/180.0);
	pathStack.push(Seg);
}

void arcRight()
{
	//fix me
	Seg.seg_number = currSeg.seg_number+1;
	Seg.seg_type = 1;
	Seg.seg_length = 4.2;
	Seg.ref_point.x = 8.27;
	Seg.ref_point.y = 14.74;
	Seg.init_tan_angle = tf::createQuaternionMsgFromYaw(-135.7*PI/180.0);
	pathStack.push(Seg);
	publisherMethod();

}

void arcLeft()
{
	//fix me
	Seg.seg_number = currSeg.seg_number+1;
	Seg.seg_type = 1;
	Seg.seg_length = 4.2;
	Seg.ref_point.x = 8.27;
	Seg.ref_point.y = 14.74;
	Seg.init_tan_angle = tf::createQuaternionMsgFromYaw(-135.7*PI/180.0);
	pathStack.push(Seg);
	publisherMethod();
}

bool checkSide(int arcAngle, int dist)
{
	if (dist > arcAngle) {
		return true;
	} else {
		return false;
	}
}

//Summary: Place a new segment on the stack after obstacle avoidance
//
void calcHalfSeg()
{
  double radius, tangentAngStart, arcAngStart, dAng, arcAng, rho;
  double tanAngle = tf::getYaw(temp_pose_out_.pose.orientation);

  //Find the distance between the end of the previous arc segment and the beginning of the new straight path segment
  lastSeg = pathStack.top();
  rho =  lastSeg.curvature;
  radius = 1.0/fabs(rho);

  tanAngleStart = tanAngle;
  arcAngStart = 0.0;
  
  if(rho >= 0.0) {
    arcAngStart = tangentAngStart - M_PI / 2.0;  
  } else {
    arcAngStart = tangentAngStart + M_PI / 2.0;

    double seg_length_done = lastSeg.length - lastSeg.ref_point.x
    dAng = seg_length_done * rho;
    arcAng = arcAngStart + dAng;

  }

    int xNewStart = 
     

    lastSeg.pop(); //take the latest path segment off of the stack
    prevSeg = pathSeg.top(); //view the previous segment
    
    pathStack.push(lastSeg); //push the last segment back to the path segment stack

    //calculate the eucledian distance between the old segment and new desired segment
    distance = sqrt(pow((lastSeg.ref_point.x - prevSeg.ref_point.x),2.0) + pow((lastSeg.ref_point.y - prevSeg.ref_point.y),2.0));
    return distance;

}

void goStraight()
{

	msg_alpha::PathSegment smallGoStraight;
	smallGoStraight.seg_number = 1;//need to increment from before obs
	smallGoStraight.seg_type = 1; //straight
	smallGoStraight.seg_length = 0.5;
	smallGoStraight.ref_point.x = last_map_pose.pose.position.x;//need to recalculate every time
	smallGoStraight.ref_point.y = last_map_pose.pose.position.y;//same
	smallGoStraight.init_tan_angle = tf::createQuaternionMsgFromYaw(-135.7*PI/180.0);//should be constant and same as pre obs angle
	pathStack.push(smallGoStraight);
	publisherMethod();
}

void publishSeg() 
{
	if (pathStack.empty()) return;
	//update currseg,pop,then pub. all in ros::ok loop
	currSeg = pathStack.top();
	while(ros::ok()){
		ros::spinOnce();
		if (segComplete == true)
		{
			pathStack.pop();
			pathPub.publish(currSeg);
			segComplete = false;
		}
	}
}

void detour()
{
	//this method assumes that the lidar node will wait three seconds
	//before publishing a segStatus of !OK

	//First thing we need to do is 
	int arcAngle;
	int obst_angle;
	int obst_side = 0; //left is 1, right is 2. this should be an enum

	ros::Subscriber obst_angle = n.subscribe<msg_alpha::Obstacles>("obstacles",1,obstaclesCallback);

	//figure out which way to turn
	if (lastObs->left_dist == 0) { //only called on abort so one should be zero and one should be distance
		obst_side = 2; //right
		arcAngle = lastObs->wall_dist_rt/2;
		arcLeft(arcAngle);
		arcRight(arcAngle);
		while(!checkSide(.6,lastObs -> rt_dist)){
			goStraight();
		}
		arcRight(arcAngle);
		arcLeft(arcAngle);

	} else {
		obst_side = 1; //left
		arcAngle = lastObs->wall_dist_lt/2;
		arcRight(arcAngle);
		arcLeft(arcAngle);
		while(!checkSide(.6,lastObs->left_dist)){
			goStraight();
		}
		arcLeft(arcAngle);
		arcRight(arcAngle);
	}
}

int main(int argc, char **argv)
{
	ros::init(argc,argv,"path_publisher");
	ros::NodeHandle n;

	msg_alpha::PathSegment Seg;

	pathPub = n.advertise<msg_alpha::PathSegment>("path_seg",1);
	ros::Subscriber segSub = n.subscribe<msg_alpha::SegStatus>("seg_status",1,segStatusCallback);

	ros::Rate naptime(10);

	while(!ros::Time::isValid()) {}	

	while(ros::ok() && !pathStack.empty())
	{
		//While seg status is ok.
		if(!abort)
		{
			ros::spinOnce();
			if(segComplete == true)
			{
				currSeg = pathStack.top();
				pathStack.pop();
				segComplete = false;
				ROS_INFO("I published another node! Be proud...");
			}
		} else {
			detour();
		}


		//ROS_INFO("Publishing!");
		pathPub.publish(currSeg);
		naptime.sleep();
	}
	return 0;
}
