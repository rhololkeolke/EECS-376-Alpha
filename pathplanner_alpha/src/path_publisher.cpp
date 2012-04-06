#include <ros/ros.h>
#include <geometry_msgs/Twist.h> //data type for velocities
#include <geometry_msgs/PoseStamped.h> //data type for Pose combined with frame and timestamp
#include <nav_msgs/Odometry.h> //data type for odometry information (see available fields with 'rosmsg show nav_msgs/Odometry')
#include <tf/transform_datatypes.h> // for tf::getYaw
#include <tf/transform_listener.h> // for the TransformListener class that abstracts away a lot of tf
#include <math.h>
#include <cmath>
#include <msg_alpha/PathSegment.h>
#include <msg_alpha/SegStatus.h>
#include <msg_alpha/Obstacles.h>
#include <iostream>
#include <stack>

const double PI=3.14159;

using namespace std;

bool segComplete = true;
bool bAbort = false ; //was formerly abort
int seg_number = 0;
msg_alpha::Obstacles lastObs;
int numSegs = 5;
msg_alpha::PathSegment currSeg;
geometry_msgs::PoseStamped last_map_pose;
tf::TransformListener *tfl;
nav_msgs::Odometry last_odom;

//Stack is created
stack <msg_alpha::PathSegment> pathStack;

ros::Publisher pathPub;
double progressMade;

geometry_msgs::PoseStamped temp;

void odomCallback(const nav_msgs::Odometry::ConstPtr& odom) 
{
        last_odom = *odom;
        temp.pose = last_odom.pose.pose;
        temp.header = last_odom.header;

        try {
        	tfl->transformPose("map", temp, last_map_pose); // given most recent odometry and most recent coord-frame transform, compute
                                                          // estimate of most recent pose in map coordinates..."last_map_pose"
        }catch (tf::TransformException ex) {
        	ROS_ERROR("%s", ex.what());
        }
}


void segStatusCallback(const msg_alpha::SegStatus::ConstPtr& status)
{
	//seg_number = status->seg_number;
	// segComplete is only true once and we only care about when segComplete transitions to true
	// so only update if the segment is currently not completed
	if(!segComplete)
		segComplete = status->segComplete;

	if(status->segComplete == false)
		ROS_INFO("CallBack false");
	else
		ROS_INFO("CallBack true");
	
	if(!bAbort)
		bAbort = status->abort;
	
}

void obstaclesCallback(const msg_alpha::Obstacles::ConstPtr& obstacles)
{
	if(obstacles->exists)
	{
		lastObs.exists = obstacles->exists;
		lastObs.distance = obstacles->distance;
		lastObs.ping_angle = obstacles->ping_angle;
	}
}

int calculateNewX(int initX, int distanceTraveled, int angle)
{

  //  int newX = cos(tf::createQuaternionMsgFromYaw(angle*PI/180.0)))*distanceTraveled; 
  int newX = initX + cos(angle*PI/180.0)*distanceTraveled; 

  return newX;
}

int calculateNewY(int initY, int distanceTraveled, int angle)
{

  //  int newY = (sin(tf::createQuaternionMsgFromYaw(angle*PI/180.0)))*distanceTraveled;
  int newY = initY + sin(angle*PI/180.0)*distanceTraveled; 
  return newY;
}


void initStack()
{
	msg_alpha::PathSegment Seg1;
	Seg1.curvature = 0; //Sets the curvature to 0 for the straight
	Seg1.max_speeds.linear.x = .25;
	Seg1.max_speeds.angular.z = .25;
	Seg1.min_speeds.linear.x = 0;
	Seg1.min_speeds.angular.z = 0;
	Seg1.accel_limit = .25;
	Seg1.decel_limit = .25;
	Seg1.seg_number = 5;
	Seg1.seg_type = 1;
	Seg1.seg_length = 1;
	Seg1.ref_point.x = calculateNewX(-3.28,1,45.22);
	Seg1.ref_point.y = calculateNewY(20.8,1,45.22);
	Seg1.init_tan_angle = tf::createQuaternionMsgFromYaw(45.22*PI/180.0);
	pathStack.push(Seg1);

	msg_alpha::PathSegment Seg2;
	Seg2.curvature = 0; //Sets the curvature to 0 for the straight
	Seg2.max_speeds.linear.x = .25;
	Seg2.max_speeds.angular.z = .25;
	Seg2.min_speeds.linear.x = 0;
	Seg2.min_speeds.angular.z = 0;
	Seg2.accel_limit = .25;
	Seg2.decel_limit = .25;
	Seg2.seg_number = 5;
	Seg2.seg_type = 1;
	Seg2.seg_length = 1;
	Seg2.ref_point.x = -3.28;
	Seg2.ref_point.y = 20.8;
	Seg2.init_tan_angle = tf::createQuaternionMsgFromYaw(45.22*PI/180.0);
	pathStack.push(Seg2);

	//Second turn
	msg_alpha::PathSegment Seg3;
	Seg3.max_speeds.linear.x = .25;
	Seg3.max_speeds.angular.z = .25;
	Seg3.min_speeds.linear.x = 0;
	Seg3.min_speeds.angular.z = 0;
	Seg3.accel_limit = .25;
	Seg3.decel_limit = .25;
	Seg3.seg_number = 5;
	Seg3.curvature = -1; //sets the curvature ot -1 for just the turn
	Seg3.seg_type = 3;
	Seg3.seg_length = -PI/2;
	Seg3.ref_point.x = 14.84;
	Seg3.ref_point.y = 3.91;
	pathStack.push(Seg3);

	msg_alpha::PathSegment Seg;
	Seg.max_speeds.linear.x = .25;
	Seg.max_speeds.angular.z = .25;
	Seg.min_speeds.linear.x = 0;
	Seg.min_speeds.angular.z = 0;
	Seg.accel_limit = .25;
	Seg.decel_limit = .25;
	Seg.curvature = 0; //sets the curvature to 0 for the entire straight
	//Long straight away from the lab
	Seg.seg_number = 3;
	Seg.seg_type = 1;
	Seg.seg_length = 1;
	Seg.ref_point.x = calculateNewX(5.45,13,136.0);
	Seg.ref_point.y = calculateNewY(11.92,13,136.0);
	Seg.init_tan_angle = tf::createQuaternionMsgFromYaw(136.0*PI/180.0);
	pathStack.push(Seg);

	Seg.seg_number = 3;
	Seg.seg_type = 1;
	Seg.seg_length = 1;
	Seg.ref_point.x = calculateNewX(5.45,12,136.0);
	Seg.ref_point.y = calculateNewY(11.92,12,136.0);
	Seg.init_tan_angle = tf::createQuaternionMsgFromYaw(136.0*PI/180.0);
	pathStack.push(Seg);

	Seg.seg_number = 3;
	Seg.seg_type = 1;
	Seg.seg_length = 1;
	Seg.ref_point.x = calculateNewX(5.45,11,136.0);
	Seg.ref_point.y = calculateNewY(11.92,11,136.0);
	Seg.init_tan_angle = tf::createQuaternionMsgFromYaw(136.0*PI/180.0);
	pathStack.push(Seg);

	Seg.seg_number = 3;
	Seg.seg_type = 1;
	Seg.seg_length = 1;
	Seg.ref_point.x = calculateNewX(5.45,10,136.0);
	Seg.ref_point.y = calculateNewY(11.92,10,136.0);
	Seg.init_tan_angle = tf::createQuaternionMsgFromYaw(136.0*PI/180.0);
	pathStack.push(Seg);

	Seg.seg_number = 3;
	Seg.seg_type = 1;
	Seg.seg_length = 1;
	Seg.ref_point.x = calculateNewX(5.45,9,136.0);
	Seg.ref_point.y = calculateNewY(11.92,9,136.0);
	Seg.init_tan_angle = tf::createQuaternionMsgFromYaw(136.0*PI/180.0);
	pathStack.push(Seg);

	Seg.seg_number = 3;
	Seg.seg_type = 1;
	Seg.seg_length = 1;
	Seg.ref_point.x = calculateNewX(5.45,8,136.0);
	Seg.ref_point.y = calculateNewY(11.92,8,136.0);
	Seg.init_tan_angle = tf::createQuaternionMsgFromYaw(136.0*PI/180.0);
	pathStack.push(Seg);

	Seg.seg_number = 3;
	Seg.seg_type = 1;
	Seg.seg_length = 1;
	Seg.ref_point.x = calculateNewX(5.45,7,136.0);
	Seg.ref_point.y = calculateNewY(11.92,7,136.0);
	Seg.init_tan_angle = tf::createQuaternionMsgFromYaw(136.0*PI/180.0);
	pathStack.push(Seg);

	Seg.seg_number = 3;
	Seg.seg_type = 1;
	Seg.seg_length = 1;
	Seg.ref_point.x = calculateNewX(5.45,6,136.0);
	Seg.ref_point.y = calculateNewY(11.92,6,136.0);
	Seg.init_tan_angle = tf::createQuaternionMsgFromYaw(136.0*PI/180.0);
	pathStack.push(Seg);

	Seg.seg_number = 3;
	Seg.seg_type = 1;
	Seg.seg_length = 1;
	Seg.ref_point.x = calculateNewX(5.45, 5,136.0);
	Seg.ref_point.y = calculateNewY(11.92,5,136.0);
	Seg.init_tan_angle = tf::createQuaternionMsgFromYaw(136.0*PI/180.0);
	pathStack.push(Seg);

	Seg.seg_number = 3;
	Seg.seg_type = 1;
	Seg.seg_length = 1;
	Seg.ref_point.x = calculateNewX(5.45,4,136.0);
	Seg.ref_point.y = calculateNewY(11.92,4,136.0);
	Seg.init_tan_angle = tf::createQuaternionMsgFromYaw(136.0*PI/180.0);
	pathStack.push(Seg);

	Seg.seg_number = 3;
	Seg.seg_type = 1;
	Seg.seg_length = 1;
	Seg.ref_point.x = calculateNewX(5.45,3,136.0);
	Seg.ref_point.y = calculateNewY(11.92,3,136.0);
	Seg.init_tan_angle = tf::createQuaternionMsgFromYaw(136.0*PI/180.0);
	pathStack.push(Seg);

	Seg.seg_number = 3;
	Seg.seg_type = 1;
	Seg.seg_length = 1;
	Seg.ref_point.x = calculateNewX(5.45,2,136.0);
	Seg.ref_point.y = calculateNewY(11.92,2,136.0);
	Seg.init_tan_angle = tf::createQuaternionMsgFromYaw(136.0*PI/180.0);
	pathStack.push(Seg);

	Seg.seg_number = 3;
	Seg.seg_type = 1;
	Seg.seg_length = 1;
	Seg.ref_point.x = calculateNewX(5.45,1,136.0);
	Seg.ref_point.y = calculateNewY(11.92,1,136.0);
	Seg.init_tan_angle = tf::createQuaternionMsgFromYaw(136.0*PI/180.0);
	pathStack.push(Seg);

	Seg.seg_number = 3;
	Seg.seg_type = 1;
	Seg.seg_length = .34;
	Seg.ref_point.x = 5.45;
	Seg.ref_point.y = 11.92;
	Seg.init_tan_angle = tf::createQuaternionMsgFromYaw(136.0*PI/180.0);
	pathStack.push(Seg);

	//Initial Turn
	Seg.seg_number = 2;
	Seg.curvature = -1; //Sets the curvature to -1 for the turn
	Seg.seg_type = 3;
	Seg.seg_length = -PI/2;
	Seg.ref_point.x = 5.23;
	Seg.ref_point.y = 11.92;		
	pathStack.push(Seg);

	
	Seg.curvature = 0; //Sets the curvature to 0 for the straight
	//Initial Straight, toward lab
	Seg.seg_number = 1;
	Seg.seg_type = 1;
	Seg.seg_length = 1;
	Seg.ref_point.x = calculateNewX(8.27,4,-135.7);
	Seg.ref_point.y = calculateNewY(14.74,4,-135.7);
	Seg.init_tan_angle = tf::createQuaternionMsgFromYaw(-135.7*PI/180.0);
	pathStack.push(Seg);

	Seg.seg_number = 4;
	Seg.seg_type = 1;
	Seg.seg_length = 1;
	Seg.ref_point.x = calculateNewX(8.27,3,-135.7);
	Seg.ref_point.y = calculateNewY(14.74,3,-135.7);
	Seg.init_tan_angle = tf::createQuaternionMsgFromYaw(-135.7*PI/180.0);
	pathStack.push(Seg);

	Seg.seg_number = 3;
	Seg.seg_type = 1;
	Seg.seg_length = 1;
	Seg.ref_point.x = calculateNewX(8.27,2,-135.7);
	Seg.ref_point.y = calculateNewY(14.74,2,-135.7);
	Seg.init_tan_angle = tf::createQuaternionMsgFromYaw(-135.7*PI/180.0);
	pathStack.push(Seg);

	Seg.seg_number = 2;
	Seg.seg_type = 1;
	Seg.seg_length = 1;
	Seg.ref_point.x = calculateNewX(8.27,1,-135.7);
	Seg.ref_point.y = calculateNewY(14.74,1,-135.7);
	Seg.init_tan_angle = tf::createQuaternionMsgFromYaw(-135.7*PI/180.0);
	pathStack.push(Seg);

	Seg.seg_number = 1;
	Seg.seg_type = 1;
	Seg.seg_length = .2;
	Seg.ref_point.x = 8.27;
	Seg.ref_point.y = 14.74;
	Seg.init_tan_angle = tf::createQuaternionMsgFromYaw(-135.7*PI/180.0);
	pathStack.push(Seg);

	Seg.seg_number = 0;
	Seg.seg_type = 1;
	Seg.seg_length = 0;
	Seg.ref_point.x = 8.27;
	Seg.ref_point.y = 14.74;
	Seg.init_tan_angle = tf::createQuaternionMsgFromYaw(-135.7*PI/180.0);
	pathStack.push(Seg);
}

void publishSeg() 
{
	ros::Rate naptime = ros::Rate(30);

	if (pathStack.empty())
		return;
	//update currseg,pop,then pub. all in ros::ok loop
	int temp = currSeg.seg_number;
	currSeg = pathStack.top();
	currSeg.seg_number = temp + 1;
	do {
		ros::spinOnce();
		ROS_INFO("Publish Seg "+segComplete);
		if (segComplete == true)
		{
			pathStack.pop();
			pathPub.publish(currSeg);
			segComplete = false;
			ROS_INFO("I published another node! Be proud... Seg number was %d", currSeg.seg_number);
		}
		naptime.sleep();
	} while((!segComplete || bAbort) && ros::ok());
}

void arcRight()
{
	int arcRadius = lastObs.wall_dist_right/2 - 20;

	msg_alpha::PathSegment Seg;
	Seg.seg_number = currSeg.seg_number+1;
	Seg.seg_type = 2;
	Seg.seg_length = (PI/2)*arcRadius;
	ROS_INFO("arcRight Hit");
	Seg.ref_point.x = last_map_pose.pose.position.x + arcRadius*cos(tf::getYaw(last_map_pose.pose.orientation));
	Seg.ref_point.y = last_map_pose.pose.position.y + arcRadius*sin(tf::getYaw(last_map_pose.pose.orientation));
	Seg.init_tan_angle = tf::createQuaternionMsgFromYaw(-PI/2);	
	pathStack.push(Seg);
	publishSeg();

}

void arcLeft()
{
	int arcRadius = lastObs.wall_dist_left/2 - 20;

	msg_alpha::PathSegment Seg;
	Seg.seg_number = currSeg.seg_number+1;
	Seg.seg_type = 2;
	Seg.seg_length = (PI/2)*arcRadius;
	ROS_INFO("arcLeft Hit");
	Seg.ref_point.x = last_map_pose.pose.position.x + arcRadius*cos(tf::getYaw(last_map_pose.pose.orientation));
	Seg.ref_point.y = last_map_pose.pose.position.y + arcRadius*sin(tf::getYaw(last_map_pose.pose.orientation));
	Seg.init_tan_angle = tf::createQuaternionMsgFromYaw(PI/2);	
	pathStack.push(Seg);
	publishSeg();

}

bool checkSide(int arcRadius, int dist)
{
	if (dist > arcRadius) {
		return true;
	} else {
		return false;
	}
}

//Summary: Place a new segment on the stack after obstacle avoidance
/*
void calcHalfSeg()
{
  
  double radius; //turn radius is inverse of curvature
  double tangentAngStart; //inital head of current segment
  double arcAngStart; //
  double dAng; //
  double arcAng; //
  double rho; //curvature
  //double tanAngle = ::getYaw(temp_pose_out_.pose.orientation);
	
  msg_alpha::PathSegment finalSeg;
  finalSeg = pathStack.top();

  rho = finalSeg.curvature;  // curvature is +/- 1/radius of arc; + for CCW trajectory    
  radius = 1.0/fabs(rho); 

	if(rho >= 0.0) {
		arcAngStart = tangentAngStart - M_PI / 2.0;  //path in polar coords
	} else {
		arcAngStart = tangentAngStart + M_PI / 2.0; //path in polar coords
	}

	dAng = progressMade * rho; //radius*delta_angle = delta_arc_distance   
	arcAng = arcAngStart + dAng; 
	double xDes = finalSeg.ref_point.x; //update desired x pos
	double yDes = finalSeg.ref_point.y; //update desired y pos
	double psiDes = tangentAngStart + dAng;

	msg_alpha::PathSegment newFinalSeg;
	newFinalSeg = pathStack.top();


	double distance = sqrt(pow((newFinalSeg.ref_point.x - finalSeg.ref_point.x),2.0) + pow((newFinalSeg.ref_point.y - finalSeg.ref_point.y),2.0));   

	msg_alpha::PathSegment newSeg;
	newSeg.seg_number = 1;
	newSeg.seg_type = 1; 
	newSeg.seg_length = distance;
	newSeg.ref_point.x = xDes;
	newSeg.ref_point.y = yDes;
	ROS_INFO("calcHalfSeg Hit");
	newSeg.init_tan_angle = tf::createQuaternionMsgFromYaw(psiDes);
	pathStack.push(newSeg);

	pathStack.push(newFinalSeg); 
}
*/

void goStraight()
{

	msg_alpha::PathSegment smallGoStraight;
	smallGoStraight.seg_number = 1;//need to increment from before obs
	smallGoStraight.seg_type = 1; //straight
	smallGoStraight.seg_length = 0.5;
	smallGoStraight.ref_point.x = 0;//need to recalculate every time
	smallGoStraight.ref_point.y = 0;//same
	ROS_INFO("goStraight Hit");
	smallGoStraight.init_tan_angle = tf::createQuaternionMsgFromYaw(-135.7*PI/180.0);//should be constant and same as pre obs angle
	pathStack.push(smallGoStraight);
	publishSeg();
}

void detour()
{
	//this method assumes that the lidar node will wait three seconds
	//before publishing a segStatus of !OK
	bAbort = false;
	//First thing we need to do is 
	int arcRadius;
	//int obst_angle;
	int obst_side = 0; //left is 1, right is 2. this should be an enum



	//figure out which way to turn
	//Is called on right turn
	if (lastObs.ping_angle < 90) { //only called on abort so one should be zero and one should be distance
		obst_side = 2; //right
		arcRadius = lastObs.wall_dist_right/2 + 20;
		arcLeft();
		arcRight();
		while(!checkSide(0.6,lastObs.wall_dist_right)){
			goStraight();
		}
		arcRight();
		arcLeft();
	//Is called on a left turn
	} else {
		obst_side = 1; //left
		arcRadius = lastObs.wall_dist_left/2;
		arcRight();
		arcLeft();
		while(!checkSide(.6,lastObs.wall_dist_left)){
			goStraight();
		}
		arcLeft();
		arcRight();
	}
}


int main(int argc, char **argv)
{
	ros::init(argc,argv,"path_publisher");
	ros::NodeHandle n;

	tfl = new tf::TransformListener();

	msg_alpha::PathSegment Seg;

	initStack();

	pathPub = n.advertise<msg_alpha::PathSegment>("path_seg",1);
	ros::Subscriber sub = n.subscribe<nav_msgs::Odometry>("odom", 1, odomCallback); 
	ros::Subscriber segSub = n.subscribe<msg_alpha::SegStatus>("seg_status",10,segStatusCallback);
	ros::Subscriber obst_angle = n.subscribe<msg_alpha::Obstacles>("obstacles",1,obstaclesCallback);

	ros::Rate naptime(30);

	pathPub.publish(pathStack.top());
	pathStack.pop();

	while(!ros::Time::isValid()) {}	

	while(ros::ok() && !pathStack.empty())
	{
		//While seg status is ok.
		if(!bAbort)
		{
			naptime.sleep();
			publishSeg();
		}
		else {
		  detour();
		}
		naptime.sleep();
	}
	return 0;
}
