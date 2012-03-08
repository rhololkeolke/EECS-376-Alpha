#include <ros/ros.h>
#include <cwru_base/Pose.h>
#include <sensor_msgs/LaserScan.h>
#include <velocity_profiler/Obstacles.h>
#include <iostream> 
#include <math.h>
#include <string>
#include <iostream>

#define _USE_MATH_DEFINES

using namespace std; 

const uint cPings = 181;
const double cBoxHeight = 1.0; //distance in front of the robot
const double cBoxWidth = 0.5; // distance from x axis to side of box (double this is width of box) 
const string cSimLaserTopic = "base_scan"; //topic to subscribe to if running code on simulator
const string cRoboLaserTopic = "base_laser1_scan"; //topic to subscribe to if running on Jinx

double curLaserData [cPings]; //lidar data

int angleSwitch; // stores the angle used to switch between distance formulas

//Summary: Receive Laser Scan Data and place it into a global array
//Parameters: Laser Scan Data
//Return: Nothing
void laserCallback(const sensor_msgs::LaserScan::ConstPtr& laserScan)
{
  {
    //curLaserData is an array created from each i element of the laserScan array
    //i == 0 .. i == the index of the final element of the laserScan array
    for(uint i = 0; i < laserScan->ranges.size(); i++)
      {
	curLaserData[i] = laserScan->ranges[i];  
      }
  }
}

//Summary: Publishes information regarding whether or not there is an obstalce within the current path segement
//Parameters: The Obstacle Publisher
//Return: Nothing
void curPath(ros::Publisher &obsPub)
{
  ros::Time time = ros::Time::now();

  velocity_profiler::Obstacles obsData;  //create an instance of the obstacle msg
  
  double closestObs = 90.0; // laser range is up to 80 so nothing should be worse than this

  //An obstacle is present if the value at the index of the iterator i is within the range of the configuaration space box
  //The iterator is always within bounds of the index of the array of laser data(cPings)
  for (uint i = 0; i < cPings;  i++)
  {
    if(i < 90-angleSwitch || i > 90+angleSwitch)
      {

	if(curLaserData[i] < cBoxWidth/cos((180.0-(double)i)*M_PI/180.0))
	  {
	    
	    if(curLaserData[i] < closestObs)
	      {
		closestObs = curLaserData[i];
	      }
	  }
      }
    else if(i > 90-angleSwitch && i < 90+angleSwitch)
      {
	
	if(curLaserData[i] < cBoxHeight/cos(((double)i-90.0)*M_PI/180.0))
	{
	
	  if(curLaserData[i] < closestObs)
	  {
	    closestObs = curLaserData[i];
	  }
	}
      }
  }
  
  if(closestObs < 90.0)
  {
    obsData.exists = true; //there is an obstacle
    obsData.distance = closestObs; //how far is the obstacle along the path
  }
  else  
  {
    obsData.exists = false; // nothing is in the way
    obsData.distance = 0.0;
  }
  obsPub.publish(obsData); // publish the data, must publish constantly or else other nodes may shutdown
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "laser_listener"); //the name of this node is laser_listener

  ros::NodeHandle n;
    
  ros::Subscriber laserSub = n.subscribe(cSimLaserTopic,1,laserCallback); //laser data comes from base_scan or base_laser1_scan, at a buffer of 1
  ros::Publisher obsPub = n.advertise<velocity_profiler::Obstacles>("obstacles",1); //obstacles is the topic to which Obstacles publishes to

  while(!ros::Time::isValid()) {} //helps simulation initialization

  angleSwitch = atan(cBoxWidth)*180/M_PI;

  ros::Rate naptime(100);  //sleeps at rate of 100Hz


  while(ros::ok())
  {
    curPath(obsPub);  //function that checks for obstacles
    
    ros::spinOnce();  //launches callbacks
    
    naptime.sleep();  //sleep for the rate of 100Hz
  }

  return 0;
}
