#include<ros/ros.h>
#include<geometry_msgs/Twist.h> //data type for velocities
#include<geometry_msgs/PoseStamped.h> //data type for Pose combined with frame and timestamp
#include<nav_msgs/Odometry.h> //data type for odometry information (see available fields with 'rosmsg show nav_msgs/Odometry')
#include<tf/transform_datatypes.h> // for tf::getYaw
#include<tf/transform_listener.h> // for the TransformListener class that abstracts away a lot of tf
#include<math.h>
#include<cmath>
#include <iostream>
#include <fstream>
#include <alpha_steering/SegStatus.h>
#include <alpha_steering/PathSegment.h>
#include <alpha_steering/Obstacles.h>
#include "lockedQueue.h"

using namespace std;

//Note that this initializes to all 0's... so until you get an "initial pose" from the first callback, it's prolly gonna be way wrong for any algorithm to use
nav_msgs::Odometry last_odom;
geometry_msgs::PoseStamped last_map_pose;
geometry_msgs::Twist des_vel;
tf::TransformListener *tfl;

int seg_number = 0;
bool segComplete = false;

// stores the values of the obstacles in the way
bool obs = false;
double obs_dist = 0.0;

// the current segment and the next segment
bool nextSegExists = false;
bool currSegExists = false;
alpha_steering::PathSegment nextSeg;
alpha_steering::PathSegment currSeg;

void obstaclesCallback(const alpha_steering::Obstacles::ConstPtr& obsData)
{
  obs = obsData->exists;
  obs_dist = obsData->distance;
}

geometry_msgs::PoseStamped temp;
void odomCallback(const nav_msgs::Odometry::ConstPtr& odom) {
        last_odom = *odom;
        temp.pose = last_odom.pose.pose;
        temp.header = last_odom.header;
	
        try {
          tfl->transformPose("map", temp, last_map_pose); // given most recent odometry and most recent coord-frame transform, compute
                                                          // estimate of most recent pose in map coordinates..."last_map_pose"
        } catch (tf::TransformException ex) {
          ROS_ERROR("%s", ex.what());
        }
}

void velCallback(const geometry_msgs::Twist::ConstPtr& vel) {
  des_vel.linear.x = vel->linear.x;
  des_vel.angular.z = vel->angular.z;
}

void pathSegCallback(const alpha_steering::PathSegment::ConstPtr& seg)
{
  if(seg->seg_number != nextSeg.seg_number) // only update if its a
					    // new segment
  {
    nextSegExists = true;
   }
  nextSeg.seg_number = seg->seg_number;
  nextSeg.seg_type = seg->seg_type;
  nextSeg.curvature = seg->curvature;
  nextSeg.seg_length = seg->seg_length;
  nextSeg.ref_point = seg->ref_point;
  nextSeg.init_tan_angle = seg->init_tan_angle;
  nextSeg.max_speeds = seg->max_speeds;
  nextSeg.accel_limit = seg->accel_limit;
  nextSeg.decel_limit = seg->decel_limit;
}

void segStatusCallback(const alpha_steering::SegStatus::ConstPtr& status)
{
  seg_number = status->seg_number;
  if(!segComplete) // only care about false to true transitions. I
		   // want to leave the segComplete true until it is
		   // processed by the loops below
  {
    segComplete = status->segComplete;
    currSegExists = false;
  }
}

int main(int argc,char **argv)
{

	ros::init(argc,argv,"alpha_steering_example");//name of this node
	ros::NodeHandle n;
        tfl = new tf::TransformListener();
	ros::Publisher pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1);
	ros::Subscriber desVelSub = n.subscribe<geometry_msgs::Twist>("des_vel",1, velCallback);
        ros::Subscriber sub = n.subscribe<nav_msgs::Odometry>("odom", 1, odomCallback); 
	ros::Subscriber path = n.subscribe<alpha_steering::PathSegment>("path_seg", 10, pathSegCallback);
	ros::Subscriber seg_status = n.subscribe<alpha_steering::SegStatus>("seg_status",10,segStatusCallback);
	ros::Subscriber obsSub = n.subscribe<alpha_steering::Obstacles>("obstacles",1,obstaclesCallback);
	//"cmd_vel" is the topic name to publish velocity commands
	//"1" is the buffer size (could use buffer>1 in case network bogs down)

	geometry_msgs::Twist vel_object;
	geometry_msgs::Twist cmd_vel;
        geometry_msgs::PoseStamped desired_pose;  // not used in this program...ideally, get by subscription
	while (!ros::Time::isValid()) {} // simulation time sometimes initializes slowly. Wait until ros::Time::now() will be valid, but let any callbacks happen
        while (!tfl->canTransform("map", "odom", ros::Time::now())) {/*ros::spinOnce();*/} // wait until there is transform data available before starting our controller loop
	//ros::Duration run_duration(3.0); // specify desired duration of this command segment to be 3 seconds
	ros::Duration elapsed_time; // define a variable to hold elapsed time
	ros::Rate naptime(10); //will perform sleeps to enforce loop rate of "10" Hz

	ros::AsyncSpinner spinner(0);
	spinner.start();

	ros::Time birthday= ros::Time::now(); // get the current time, which defines our start time, called "birthday"
	ROS_INFO("birthday started as %f", birthday.toSec());

	double xf, yf, xs, ys; // Final desired x,y values for the current leg of the move

	vel_object.linear.x = 0.0; // Initial velocity
	vel_object.angular.z = 0.0;
	
	double dtheta, offset;
 	double desired_heading,current_heading; //desired and actual robot heading
	double x_current=0.0,y_current=0.0; // current x,y position of robot in map coords
	double tx,ty,nx,ny;  //path tangent and normal vector components
	double xrs,yrs; // x and y coords of robot relative to start (xs,ys)
	// tune these values, Kd and Ktheta, for alpha_steering
	double Kd =0.5;
	double Ktheta = 1.0;
	
	// open config file for reading.
	//cout << argv[0] << endl;
	string base = argv[0];
	int path_loc = base.find("bin/");
	string new_part = "config/alpha_steering_constants.txt";
	base.replace(path_loc,29,new_part);
	//cout << "base: " << base << endl;
	
	std::ifstream infile(base.c_str());
	//infile.open("../config/alpha_steering_constants.txt");
	if(infile.good()){
		infile >> Kd;
		infile >> Ktheta;
	} else {
		cout << "there was a problem reading the file for input" << endl;
	}
	infile.close();
	
	double dt = .05;
	double pi = 3.14159;

	ROS_INFO("desired heading: %f",desired_heading);
	
	bool lastobs = false;
	double obsDist = 0.0;
	double decel_rate;

	double xStart,yStart;

	while (ros::ok()) // do work here
	{
	  if(!currSegExists) // while there is no current segment
	  {
	    if(nextSegExists) // see if there are any new segments
	    {
	      currSegExists = true;
	      nextSegExists = false;
	      currSeg = nextSeg;
	      segComplete = false;
	      if(currSeg.seg_type == 1)
	      {
		  xs = currSeg.ref_point.x; // get the start point
		  ys = currSeg.ref_point.y;

		  desired_heading = tf::getYaw(currSeg.init_tan_angle); // get the path's heading
	      
		  xf = xs + currSeg.seg_length*cos(desired_heading); // get the final point
		  yf = ys + currSeg.seg_length*sin(desired_heading);
	      }
	    }
	  }
    
	  if(!segComplete) // make sure we are alpha_steering to the same line
	  {
	    if(currSeg.seg_type == 1) // straight lines, so far enable alpha_steering only for straights
	    {
	      if(obs && !lastobs)
		{
		  ROS_INFO("line 219");
		  //ROS_INFO("Obstacles: Initializing obstacles");
	       	  xStart = x_current;
		  yStart = y_current;
		  obsDist = obs_dist;
		  lastobs = true;
		  decel_rate = pow(cmd_vel.linear.x,2)/(obs_dist-.1);
		  // cout << "Obstacles: obs_dist: " << obs_dist << ", decel_rate: " << decel_rate << endl; 
		}
	      
	      if(lastobs && obs && sqrt(pow(xStart-xf,2)+pow(yStart-yf,2)) >= obs_dist) // there is an obstacle in the way
		{
		  // calculate the deceleration rate 
		  if(cmd_vel.linear.x > .001 && obs_dist > .2)
		    {
		      cmd_vel.linear.x = cmd_vel.linear.x - decel_rate*dt;
		    }
		  else
		    {
		      cmd_vel.linear.x = 0.0;
		    }
		  
		  cmd_vel.angular.z = 0.0;
		  pub.publish(cmd_vel);
		  
		  naptime.sleep();
		  continue;
		}
	      else
	      {
		  lastobs = false;
	      }

		ros::Time current_time = ros::Time::now();
		elapsed_time= current_time-birthday;
                
                
		ROS_INFO("\n\n");
		ROS_INFO("time from birthday: %f", (last_map_pose.header.stamp - birthday).toSec());


		current_heading = tf::getYaw(last_map_pose.pose.orientation);
		//corresponding tangent and normal vector components of reference path (straight-line example)
		tx =cos(desired_heading);
		ty = sin(desired_heading);
		nx = -ty;
		ny = tx;
		

		// error in heading
		dtheta = desired_heading-current_heading;
		if (dtheta>pi)   // watch out for wrap-around
			dtheta=dtheta-2*pi;
		if (dtheta<-3.14159)
                        dtheta=dtheta+2*pi;

                //compute offset error:
		x_current = last_map_pose.pose.position.x;
		y_current = last_map_pose.pose.position.y;

		//vector from start point to current robot point
		xrs = x_current-xs;
		yrs = y_current-ys;

		// dot this vector with path normal vector to get the offset (works for line segments)
		offset = xrs*nx+yrs*ny;
		
		// alpha_steering control law
		cmd_vel.angular.z = -Kd*offset +Ktheta*dtheta; // simple, linear controller; can do better
		cmd_vel.linear.x=0.5; //command constant fwd vel
			
		// saturate around desired velocities
	        if(des_vel.linear.x < 0.001 && des_vel.linear.x > -0.001) {
			cmd_vel.linear.x == 0.0;
		} else if (cmd_vel.linear.x > 1.25*des_vel.linear.x) {
			cmd_vel.linear.x == 1.25*des_vel.linear.x;
		} else if (cmd_vel.linear.x > 0.75*des_vel.linear.x) {
			cmd_vel.linear.x == 0.75*des_vel.linear.x;
			}
		pub.publish(cmd_vel); // Publish the velocity (incorporating feedback)
		naptime.sleep(); //Sleep, thus enforcing the desired update rate
	    }
	    else
	    {
	      pub.publish(des_vel); // no control so simply published the desired velocity
	    }
	  }
	}// while
	delete tfl;
	return 0; // this code will only get here if this node was told to shut down
}// main
