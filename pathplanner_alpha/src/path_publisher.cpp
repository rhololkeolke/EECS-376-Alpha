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

//Stack is created
stack <msg_alpha::PathSegment> pathStack;

ros::Publisher pathPub;
double progressMade;


void segStatusCallback(const msg_alpha::SegStatus::ConstPtr& status)
{
	//seg_number = status->seg_number;
	// segComplete is only true once and we only care about when segComplete transitions to true
	// so only update if the segment is currently not completed
	if(!segComplete)

	{
	  segComplete = status->segComplete;
	}
	
	bAbort = status->abort;
	
	  {
		segComplete = status->segComplete;
		progressMade = status-> progress_made;
		
	  }
	

}

void obstaclesCallback(const msg_alpha::Obstacles::ConstPtr& obstacles)
{
	if(obstacles->exists)
	{
		lastObs = obstacles->distance;
	}
}

int calculateNewX(int initX, int distanceTraveled, int angle)
{

  int newX = (cos(double((tf::createQuaternionMsgFromYaw(angle*PI/180.0))))))))*distanceTraveled; //compiler complained about no being dbl idk if my change is right
  return newX;

}

int calculateNewY(int initY, int distanceTraveled, int angle)
{

  int newY = (sin(double(tf::createQuaternionMsgFromYaw(angle*PI/180.0)))))))*distanceTraveled;
  return newY;

}


void initStack()
{

	msg_alpha::PathSegment Seg;		

	//To the Coffee Machine
	Seg.seg_number = 5;
	Seg.seg_type = 1;
	Seg.seg_length = 1;
	Seg.ref_point.x = calculateNewX(-3.28,1,45.22);
	Seg.ref_point.y = calculateNewY(20.8,1,45.22);
	Seg.init_tan_angle = tf::createQuaternionMsgFromYaw(45.22*PI/180.0);
	pathStack.push(Seg);

	Seg.seg_number = 5;
	Seg.seg_type = 1;
	Seg.seg_length = 1;
	Seg.ref_point.x = -3.28;
	Seg.ref_point.y = 20.8;
	Seg.init_tan_angle = tf::createQuaternionMsgFromYaw(45.22*PI/180.0);
	pathStack.push(Seg);

	//Second turn
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
	Seg.seg_type = 3;
	Seg.seg_length = -PI/2;
	Seg.ref_point.x = 5.23;
	Seg.ref_point.y = 11.92;		
	pathStack.push(Seg);

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
}

void publishSeg() 
{
	if (pathStack.empty())
		return;
	//update currseg,pop,then pub. all in ros::ok loop
	int temp = currSeg.seg_number;
	currSeg = pathStack.top();
	currSeg.seg_number = temp + 1;
	do {
		ros::spinOnce();
		if (segComplete == true)
		{
			pathStack.pop();
			pathPub.publish(currSeg);
			segComplete = false;
			ROS_INFO("I published another node! Be proud...");
		}
	} while(!segComplete);
}


void arcRight(int angle)
{
	msg_alpha::PathSegment Seg;
	//fix me
	Seg.seg_number = currSeg.seg_number+1;
	Seg.seg_type = 1;
	Seg.seg_length = 4.2;
	Seg.ref_point.x = 8.27;
	Seg.ref_point.y = 14.74;
	//This needs attention
	Seg.init_tan_angle = tf::createQuaternionMsgFromYaw(angle);
	pathStack.push(Seg);
	publishSeg();

}

void arcLeft(int angle)
{
	msg_alpha::PathSegment Seg;	
	//fix me
	Seg.seg_number = currSeg.seg_number+1;
	Seg.seg_type = 1;
	Seg.seg_length = 4.2;
	Seg.ref_point.x = 8.27;
	Seg.ref_point.y = 14.74;
	//this needs attention
	Seg.init_tan_angle = tf::createQuaternionMsgFromYaw(angle);
	pathStack.push(Seg);
	publishSeg();
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

void calcHalfSeg()
{
  
  double radius; //turn radius is inverse of curvature
  double tangentAngStart; //inital head of current segment
  double arcAngStart; //
  double dAng; //
  double arcAng; //
  double rho; //curvature
  //double tanAngle = tf::getYaw(temp_pose_out_.pose.orientation);
	
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
/*
	msg_alpha::PathSegment seg;
	seg.seg_number = 2;//need to increment from before obs                                                                                                 
	seg.seg_type = 1; //straight                                                                                                                           
	seg.seg_length = ?;
	seg.ref_point.x = xDes;
	seg.ref_point.y = yDes;
	seg.init_tan_angle = tf::createQuaternionMsgFromYaw(psiDes);
	pathStack.push(seg);
*/
	msg_alpha::PathSegment newFinalSeg;
	newFinalSeg = pathStack.top();


	double distance = sqrt(pow((newFinalSeg.ref_point.x - finalSeg.ref_point.x),2.0) + pow((newFinalSeg.ref_point.y - finalSeg.ref_point.y),2.0));   

	msg_alpha::PathSegment newSeg;
	newSeg.seg_number = 1;
	newSeg.seg_type = 1; 
	newSeg.seg_length = distance;
	newSeg.ref_point.x = xDes;
	newSeg.ref_point.y = yDes;
	newSeg.init_tan_angle = tf::createQuaternionMsgFromYaw(psiDes);
	pathStack.push(newSeg);

	pathStack.push(newFinalSeg); 

}


void goStraight()
{

	msg_alpha::PathSegment smallGoStraight;
	smallGoStraight.seg_number = 1;//need to increment from before obs
	smallGoStraight.seg_type = 1; //straight
	smallGoStraight.seg_length = 0.5;
	smallGoStraight.ref_point.x = 0;//need to recalculate every time
	smallGoStraight.ref_point.y = 0;//same
	smallGoStraight.init_tan_angle = tf::createQuaternionMsgFromYaw(-135.7*PI/180.0);//should be constant and same as pre obs angle
	pathStack.push(smallGoStraight);
	publishSeg();
}

void detour()
{
	//this method assumes that the lidar node will wait three seconds
	//before publishing a segStatus of !OK

	//First thing we need to do is 
	int arcAngle;
	int obst_angle;
	int obst_side = 0; //left is 1, right is 2. this should be an enum



	//figure out which way to turn
	if (lastObs.left_dist == 0) { //only called on abort so one should be zero and one should be distance
		obst_side = 2; //right
		arcAngle = lastObs.wall_dist_rt/2;
		arcLeft(arcAngle);
		arcRight(arcAngle);
		while(!checkSide(0.6,lastObs.rt_dist)){
			goStraight();
		}
		arcRight(arcAngle);
		arcLeft(arcAngle);

	} else {
		obst_side = 1; //left
		arcAngle = lastObs.wall_dist_lt/2;
		arcRight(arcAngle);
		arcLeft(arcAngle);
		while(!checkSide(.6,lastObs.left_dist)){
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
	ros::Subscriber obst_angle = n.subscribe<msg_alpha::Obstacles>("obstacles",1,obstaclesCallback);

	ros::Rate naptime(10);

	while(!ros::Time::isValid()) {}	

	while(ros::ok() && !pathStack.empty())
	{
		//While seg status is ok.
		if(!abort)
		{
			publishSeg();
		}
		else {
		  detour();
		}
		naptime.sleep();
	}
	return 0;
}
