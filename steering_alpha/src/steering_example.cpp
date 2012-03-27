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
#include <msg_alpha/SegStatus.h>
#include <msg_alpha/PathSegment.h>
#include <msg_alpha/Obstacles.h>
#include "lockedQueue.h"

using namespace std;

//Note that this initializes to all 0's... so until you get an "initial pose" from the first callback, it's prolly gonna be way wrong for any algorithm to use
nav_msgs::Odometry last_odom;
geometry_msgs::PoseStamped last_map_pose;
geometry_msgs::Twist des_vel;
tf::TransformListener *tfl;

//lockedQueue<msg_alpha::PathSegment*> segments;

int seg_number = 0;
bool segComplete = false;

// stores the values of the obstacles in the way
bool obs = false;
double obs_dist = 0.0;

// the current segment and the next segment
bool nextSegExists = false;
bool currSegExists = false;
msg_alpha::PathSegment nextSeg;
msg_alpha::PathSegment currSeg;

void obstaclesCallback(const msg_alpha::Obstacles::ConstPtr& obsData)
{
  obs = obsData->exists;
  obs_dist = obsData->distance;
}

geometry_msgs::PoseStamped temp;
void odomCallback(const nav_msgs::Odometry::ConstPtr& odom) {
        last_odom = *odom;
        temp.pose = last_odom.pose.pose;
        temp.header = last_odom.header;
	
	//cout << "pose " << temp.pose << ", header " << temp.header << endl;
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

void pathSegCallback(const msg_alpha::PathSegment::ConstPtr& seg)
{
  if(seg->seg_number != nextSeg.seg_number)
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

void segStatusCallback(const msg_alpha::SegStatus::ConstPtr& status)
{
  seg_number = status->seg_number;
  if(!segComplete)
  {
    segComplete = status->segComplete;
    currSegExists = false;
  }
}

int main(int argc,char **argv)
{

	ros::init(argc,argv,"msg_alpha_example");//name of this node
	ros::NodeHandle n;
        tfl = new tf::TransformListener();
	ros::Publisher pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1);
	ros::Subscriber desVelSub = n.subscribe<geometry_msgs::Twist>("des_vel",1, velCallback);
        ros::Subscriber sub = n.subscribe<nav_msgs::Odometry>("odom", 1, odomCallback); 
	ros::Subscriber path = n.subscribe<msg_alpha::PathSegment>("path_seg", 10, pathSegCallback);
	ros::Subscriber seg_status = n.subscribe<msg_alpha::SegStatus>("seg_status",10,segStatusCallback);
	ros::Subscriber obsSub = n.subscribe<msg_alpha::Obstacles>("obstacles",1,obstaclesCallback);
	//"cmd_vel" is the topic name to publish velocity commands
	//"1" is the buffer size (could use buffer>1 in case network bogs down)

	geometry_msgs::Twist vel_object;
	geometry_msgs::Twist cmd_vel;
        geometry_msgs::PoseStamped desired_pose;  // not used in this program...ideally, get by subscription
	while (!ros::Time::isValid()) {/*ros::spinOnce();*/} // simulation time sometimes initializes slowly. Wait until ros::Time::now() will be valid, but let any callbacks happen
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
	// tune these values, Kd and Ktheta, for msg_alpha
	double Kd =0.5;
	double Ktheta = 1.0;
	
	// open config file for reading.
	//cout << argv[0] << endl;
	string base = argv[0];
	int path_loc = base.find("bin/");
	string new_part = "config/msg_alpha_constants.txt";
	base.replace(path_loc,29,new_part);
	//cout << "base: " << base << endl;
	
	std::ifstream infile(base.c_str());
	//infile.open("../config/msg_alpha_constants.txt");
	if(infile.good()){
		infile >> Kd;
		infile >> Ktheta;
	} else {
		cout << "there was a problem reading the file for input" << endl;
	}
	infile.close();
	
	double dt = .05;
	double pi = 3.14159;

        // instead of subscribing to path segments, for this simple example, hard-code a single line
	/*xs = 5.12; // start point
	ys = 12.04;

	xf = -16.58; // end point (or a second point, forward, on the line)
	yf = 33.59;
	desired_heading = atan2(yf-ys,xf-xs);*/
	ROS_INFO("desired heading: %f",desired_heading);
	
	bool lastobs = false;
	double obsDist = 0.0;
	double decel_rate;

	double xStart,yStart;

	while (ros::ok()) // do work here
	{
	  ROS_INFO("line 174");
	  if(!currSegExists) // while there is no current segment
	  {
	    ROS_INFO("line 177");
	    if(nextSegExists) // see if there are any new segments
	    {
	      ROS_INFO("line 180");
	      currSegExists = true;
	      nextSegExists = false;
	      currSeg = nextSeg;
	      segComplete = false;
	      ROS_INFO("line 185");
	      if(currSeg.seg_type == 1)
	      {
		ROS_INFO("line 188");
		  xs = currSeg.ref_point.x; // get the start point
		  ys = currSeg.ref_point.y;

		  desired_heading = tf::getYaw(currSeg.init_tan_angle); // get the path's heading
	      
		  xf = xs + currSeg.seg_length*cos(desired_heading); // get the final point
		  yf = ys + currSeg.seg_length*sin(desired_heading);
	      }
	      if(currSeg.seg_type == 2)
		  {
			desired_heading = PSIDES;
		  }
	    }
	    /*else
	    {
	      ROS_INFO("line 200");
	      vel_object.linear.x = 0.0;
	      vel_object.angular.z = 0.0;
	      pub.publish(vel_object);
	      continue; // nothing to do start again
	      }*/
	  }
	  
	  ROS_INFO("segComplete %i",segComplete);
	  ROS_INFO("nextSegExists %i",nextSegExists);
	  ROS_INFO("currSegExists %i", currSegExists);
	  if(!segComplete) // make sure we are msg_alpha to the same line
	  {
	    ROS_INFO("line 213");
	    if(currSeg.seg_type == 1) // straight lines, so far enable msg_alpha only for straights
	    {
	      ROS_INFO("line 216");
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
		  ROS_INFO("line_231");
		  //ROS_INFO("Running obstacle code");
		  // calculate the deceleration rate
		  
		  if(cmd_vel.linear.x > .001 && obs_dist > .2)
		    {
		      cmd_vel.linear.x = cmd_vel.linear.x - decel_rate*dt;
		      ROS_INFO("Obstacles: cmd_vel: %f", cmd_vel.linear.x);
		    }
		  else
		    {
		      cmd_vel.linear.x = 0.0;
		      ROS_INFO("Obstacles: I should be stopped");
		    }
		  
		  ROS_INFO("line 246");		  
		  cmd_vel.angular.z = 0.0;
		  pub.publish(cmd_vel);
		  
		  ROS_INFO("About to take a nap");
		  naptime.sleep();
		  continue;
		}
	      else
	      {
		  ROS_INFO("Obstacles: Obstacle is now gone");
		  lastobs = false;
	      }

	      ROS_INFO("line 260");
	      //ros::spinOnce(); // allow any subscriber callbacks that have been queued up to fire, but don't spin infinitely
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
		
		ROS_INFO("line 277");

		// error in heading
		dtheta = desired_heading-current_heading;
		if (dtheta>pi)   // watch out for wrap-around
			dtheta=dtheta-2*pi;
		if (dtheta<-3.14159)
                        dtheta=dtheta+2*pi;

		ROS_INFO("line 286");
                //compute offset error:
		x_current = last_map_pose.pose.position.x;
		y_current = last_map_pose.pose.position.y;

		ROS_INFO("line 291");
		//vector from start point to current robot point
		xrs = x_current-xs;
		yrs = y_current-ys;

		ROS_INFO("line 296");
		// dot this vector with path normal vector to get the offset (works for line segments)
		offset = xrs*nx+yrs*ny;
		
		ROS_INFO("line 300");
		// msg_alpha control law
		cmd_vel.angular.z = -Kd*offset +Ktheta*dtheta; // simple, linear controller; can do better
		cmd_vel.linear.x=0.5; //command constant fwd vel
		
		
		//ROS_INFO("Offset=%f, dtheta=%f, cmd omega=%f",offset,dtheta,cmd_vel.angular.z);
		//cout << "variables " << Kd << ", " << Ktheta << endl;
	
		// saturate around desired velocities
		ROS_INFO("line 310");
	        if(des_vel.linear.x < 0.001 && des_vel.linear.x > -0.001) {
		  ROS_INFO("line 312");
			cmd_vel.linear.x == 0.0;
		} else if (cmd_vel.linear.x > 1.25*des_vel.linear.x) {
		  ROS_INFO("line 315");
			cmd_vel.linear.x == 1.25*des_vel.linear.x;
		} else if (cmd_vel.linear.x > 0.75*des_vel.linear.x) {
		  ROS_INFO("line 318");
			cmd_vel.linear.x == 0.75*des_vel.linear.x;
			}
		ROS_INFO("line 321");
		//cmd_vel.linear.x = des_vel.linear.x;
		pub.publish(cmd_vel); // Publish the velocity (incorporating feedback)
		ROS_INFO("Published");
		naptime.sleep(); //Sleep, thus enforcing the desired update rate
		ROS_INFO("Good morning");
	    }
	    else
	    {
	      ROS_INFO("line 330");
	      pub.publish(des_vel); // no control so simply published the desired velocity
	    }
	  }
	}// while
	ROS_INFO("line 335");
	delete tfl;
	return 0; // this code will only get here if this node was told to shut down
}// main
