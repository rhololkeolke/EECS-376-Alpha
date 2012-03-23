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
const uint cRadius = 0.5;
const double cBoxHeight = 1.0; //distance in front of the robot
const double cBoxWidth = 0.5; // distance from x axis to side of box (double this is width of box) 
const string cSimLaserTopic = "base_scan"; //topic to subscribe to if running on Jinx
const string cRoboLaserTopic = "base_laser1_scan"; //topic to subscribe to if running on Jinx



int seg_type;
double scanData [cPings]; //lidar data

int angleSwitch; // stores the angle used to switch between distance formulas

//Summary: Receive Laser Scan Data and place it into a global array
//Parameters: Laser Scan Data
//Return: Nothing
void laserCallback(const sensor_msgs::LaserScan::ConstPtr& laserScan)
{
  {
    //scanData is an array created from each i element of the laserScan array
    //i == 0 .. i == the index of the final element of the laserScan array
    for(uint i = 0; i < laserScan->ranges.size(); i++)
      {
	scanData[i] = laserScan->ranges[i];  
      }
  }
}


void pathSegCb(const velocity_profiler::PathSegment::ConstPtr& seg)
{
  
  seg_number = seg->seg_number;
  seg_type = seg->seg_type;
  curvature = seg->curvature;
  seg_length = seg->seg_length;
  ref_point = seg->ref_point;
  init_tan_angle = seg->init_tan_angle;
}









//Summary: Publishes information regarding whether or not there is an obstalce within the current path segement
//Parameters: The Obstacle Publisher
//Return: Nothing
void straight(ros::Publisher &obsPub)
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

	if(scanData[i] < cBoxWidth/cos((180.0-(double)i)*M_PI/180.0))
	  {
	    
	    if(scanData[i] < closestObs)
	      {
		closestObs = scanData[i];
	      }
	  }
      }
    else if(i > 90-angleSwitch && i < 90+angleSwitch)
      {
	
	if(scanData[i] < cBoxHeight/cos(((double)i-90.0)*M_PI/180.0))
	{
	
	  if(scanData[i] < closestObs)
	  {
	    closestObs = scanData[i];
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

void arc(ros::Publisher &obsPub)
{


  //Enlarge each lidar ping. Create a circle around each ping point with constant radius. If the point along the arced path is equal to a point on the circle then an obstacle exists
  for(uint i = 0; i < cPings; i++)
    {
      //make a radius from each ping
      endPoint = scanData[i] + cRadius;
      
    }
}


void spin(ros::Publisher &obsPub)
{

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

    //if the segment is a straight path check for obstacles along the straight path
    if(seg_type == 1)
      {
	straight(obsPub);  
      }

    //if the segement is an arcd path check for obstacles along the arc path
    if(seg_type == 2)
      {
	arc(obsPub);
      }

    //if the segement is a spin in place check for obstacles alongs the path
    if(seg_type == 3)
      {
	spin(obsPub);
      }
  }
   
  ros::spinOnce();  //launches callbacks
  
  naptime.sleep();  //sleep for the rate of 100Hz

  return 0;
}
