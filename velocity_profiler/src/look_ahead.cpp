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
const string cLaserTopic = "base_scan"; 

double curLaserData [cPings]; //current path lidar info

int angleSwitch; // stores the angle used to switch between distance formulas

//Summary: Receive scan for the current path which is 1m and the next path which is the second meter and place them into accessible arrays
//Parameters: Laser Scan Data
//Return: Nothing
//Todo: Determine if scan ranges are appropriate
void laserCallback(const sensor_msgs::LaserScan::ConstPtr& laserScan)
{
  {
    for(uint i = 0; i < laserScan->ranges.size(); i++)
      {
	curLaserData[i] = laserScan->ranges[i];
      }
  }
}

//Summary: Publishes information regarding whether or not there is an obstalce within the current path segement
//Parameters: The Obstacle Publisher
//Return: Nothing
//Todo: Implement SegStatus msg type

void curPath(ros::Publisher &obsPub)
{
  ros::Time time = ros::Time::now();

  velocity_profiler::Obstacles obsData;  //create an instance of the obstacle msg
  
  double closestObs = 90.0; // laser range is up to 80 so nothing should be worse than this
  
  for (uint i = 0; i < cPings;  i++)
  {
    //    cout << "ping: " << i << endl;
      if(i < 90-angleSwitch || i > 90+angleSwitch)
      {
	//	cout << "\ti<90-" << angleSwitch << " || i>90+" << angleSwitch << endl;
	if(curLaserData[i] < cBoxWidth/cos((180.0-(double)i)*M_PI/180.0))
	{
	  //cout << "\t\tcurLaserData[" << i << "]: " << curLaserData[i] << " < " << cBoxWidth/cos((180.0-(double)i)*M_PI/180.0) << endl;
	  if(curLaserData[i] < closestObs)
	  {
	    closestObs = curLaserData[i];
	  }
	}
      }
      else if(i > 90-angleSwitch && i < 90+angleSwitch)
      {
	//cout << "\ti>90-" << angleSwitch << " && i < 90+" << angleSwitch << endl;
	if(curLaserData[i] < cBoxHeight/cos(((double)i-90.0)*M_PI/180.0))
	{
	  // cout << "\t\tcurLaserData[" << i << "]: " << curLaserData[i] << " < " << cBoxHeight/cos(((double)i-90.0)*M_PI/180.0) << endl;
	  if(curLaserData[i] < closestObs)
	  {
	    closestObs = curLaserData[i];
	  }
	}
      }
  }
  
  if(closestObs < 90.0)
  {
    obsData.exists = true;
    obsData.distance = closestObs;
  }
  else  
  {
    obsData.exists = false; // nothing is in the way
    obsData.distance = 0.0;
  }
  obsPub.publish(obsData); // publish the data
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "laser_listener");

  ros::NodeHandle n;
    
  ros::Subscriber laserSub = n.subscribe(cLaserTopic,1,laserCallback);
  ros::Publisher obsPub = n.advertise<velocity_profiler::Obstacles>("obstacles",1);

  while(!ros::Time::isValid()) {}

  angleSwitch = atan(cBoxWidth)*180/M_PI;

  ros::Rate naptime(100);  
  while(ros::ok())
  {
      curPath(obsPub);

      ros::spinOnce();

      naptime.sleep();
  }

  return 0;
}
