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

using namespace std;

//Note that this initializes to all 0's... so until you get an "initial pose" from the first callback, it's prolly gonna be way wrong for any algorithm to use
nav_msgs::Odometry last_odom;
geometry_msgs::PoseStamped last_map_pose;
tf::TransformListener *tfl;

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

int main(int argc,char **argv)
{
	ros::init(argc,argv,"steering_example");//name of this node
	ros::NodeHandle n;
        tfl = new tf::TransformListener();
	ros::Publisher pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1);
        ros::Subscriber sub = n.subscribe<nav_msgs::Odometry>("odom", 1, odomCallback); 
	//"cmd_vel" is the topic name to publish velocity commands
	//"1" is the buffer size (could use buffer>1 in case network bogs down)

	geometry_msgs::Twist vel_object;
	geometry_msgs::Twist cmd_vel;
        geometry_msgs::PoseStamped desired_pose;  // not used in this program...ideally, get by subscription
	while (!ros::Time::isValid()) {ros::spinOnce();} // simulation time sometimes initializes slowly. Wait until ros::Time::now() will be valid, but let any callbacks happen
        while (!tfl->canTransform("map", "odom", ros::Time::now())) {ros::spinOnce();} // wait until there is transform data available before starting our controller loop
	//ros::Duration run_duration(3.0); // specify desired duration of this command segment to be 3 seconds
	ros::Duration elapsed_time; // define a variable to hold elapsed time
	ros::Rate naptime(10); //will perform sleeps to enforce loop rate of "10" Hz
	ros::Time birthday= ros::Time::now(); // get the current time, which defines our start time, called "birthday"
	ROS_INFO("birthday started as %f", birthday.toSec());

	double xf, yf, xs, ys; // Final desired x,y values for the current leg of the move

	vel_object.linear.x = 0.0; // Initial velocity
	vel_object.angular.z = 0.0;
	
	double dtheta, offset;
 	double desired_heading,current_heading; //desired and actual robot heading
	double x_current,y_current; // current x,y position of robot in map coords
	double tx,ty,nx,ny;  //path tangent and normal vector components
	double xrs,yrs; // x and y coords of robot relative to start (xs,ys)
	// tune these values, Kd and Ktheta, for steering
	double Kd =0.5;
	double Ktheta = 1.0;
	
	// open config file for reading.
	std::ifstream infile;
	infile.open("../config/steering_constants.txt");
	if(infile.good() && !infile.bad()){
		infile >> Kd;
		infile >> Ktheta;
	} else {
		cout << "there was a problem reading the file for input" << endl;
	}
	infile.close();
	
	double dt = .05;
	double pi = 3.14159;
        // instead of subscribing to path segments, for this simple example, hard-code a single line
	xs = 5.12; // start point
	ys = 12.04;

	xf = -16.58; // end point (or a second point, forward, on the line)
	yf = 33.59;
	desired_heading = atan2(yf-ys,xf-xs);
	ROS_INFO("desired heading: %",desired_heading);
	
	while (ros::ok()) // do work here
	{
		ros::spinOnce(); // allow any subscriber callbacks that have been queued up to fire, but don't spin infinitely
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
		
		// steering control law
		cmd_vel.angular.z = -Kd*offset +Ktheta*dtheta; // simple, linear controller; can do better
		cmd_vel.linear.x=0.5; //command constant fwd vel
		
		
		ROS_INFO("Offset=%f, dtheta=%f, cmd omega=%f",offset,dtheta,cmd_vel.angular.z);
		
		
		pub.publish(cmd_vel); // Publish the velocity (incorporating feedback)
		
		naptime.sleep(); //Sleep, thus enforcing the desired update rate
	}// while
	
  delete tfl;
	return 0; // this code will only get here if this node was told to shut down
}// main
