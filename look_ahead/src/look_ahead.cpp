#include <ros/ros.h>
#include <cwru_base/Pose.h>
#include <sensor_msgs/LaserScan.h>
#include<look_ahead/Obstacles.h>
#include<look_ahead/PathVector.h>
#include <iostream> 



using namespace std;

const int cPings 181
const double cNotificationDistance = 0.75; //distance at which to publish that an obstacle exists for current path
const double cNotificationDistanceNext = 1.75; //distance at which to publish that an obstacle exists for next path


double curLaserData [cPings]; //current path lidar info
double nextLaserData [cPings]; //next path lidar info

//Summary: Publishes information regarding whether or not there is an obstalce within the current path segement
//Parameters: The Obstacle Publisher
//Return: Nothing
//Todo: Implement SegStatus msg type

void curPath(ros::Publisher &obsPub)
{
  look_ahead::Obstacles obsData;  //create an instance of the obstacle msg
  
  for (uint i = 0; i < cPings;  i++)
    {
      if(curLaserData[i] < cNotificationDistance)
	{
	  obsData.curExists = true; //boolean for obstacle existance
	  obsData.curDistance = cNotificationDistance; //obstacle distance
	  
	  obsPub.publish(obsData);
	}

      obsData.curExists = false;
      obsData.curDistance = 0.0;
      obsPub.publish(obsData);
    }
}

//Summary: Publishes whether or not an obstalce exists within the next path segment
//Parameters: The obstacle publisher
//Return: Nothing
//Todo: Implement SegStatus msg

void nextPath(ros::Publisher &obsPub)
{
  look_ahead::Obstacles obsData;

  while(ros::ok()) 
    {
      for(uint i = 0; i < cPings; i++)
	{
	  if(nextLaserData[i] < cNotificationDistanceNext)
	    {
	      obsData.nextExists = true;
	      obsData.nextDistance = cNotificationDistanceNext;
	    }
	  obsData.nextExists = false;
	  obsData.nextDistance = 0.0;
	  
	}
    }
}


//Summary: Receive scan for the current path which is 1m and the next path which is the second meter and place them into accessible arrays
//Parameters: Laser Scan Data
//Return: Nothing
//Todo: Determine if scan ranges are appropriate

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& laserScan)
{
    
  //Scan 0m to 1m and place data into current path data array
  while(laserScan->range_max < 1)
     {
       for(uint i = 0; i < laserScan->ranges.size(); i++)
	 {
	   curLaserData[i] = laserScan->ranges[i];
	 }
     }
  
  
  //Scan 1m to 2m and place data into next path data array
  while(laserScan->range_min >= 1 && laserScan->range_max < 2)   
    {
      for(uint j = 0; j < laserScan->ranges.size(); j++)
	{
	  nextLaserData[j] = laserScan->ranges[j];
	}
    }
  
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "laser_listener");

    ros::NodeHandle n;
    
    ros::Subscriber laserSub = n.subscribe("base_scan",1,laserCallback);
    ros::Publisher obsPub = n.advertise<look_ahead::Obstacles>("obstacles",1);

    while(!ros::Time::isValid()) {}

      
      curPath(obsPub);
      nextPath(obsPub);
      

    
    ros::spin();

    return 0;
}

