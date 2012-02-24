#include <ros/ros.h>
#include <cwru_base/Pose.h>
#include <sensor_msgs/LaserScan.h>
#include<look_ahead/Obstacles.h>
#include<look_ahead/PathVector.h>
#include <iostream>


using namespace std;

const double cNotificationDistance = 0.75; //distance at which to publish that an obstacle exists
const double cNotificationDistanceNext = 1.75;


double curLaserData [181]; //lidar data for path is copied into this array 
double nextLaserData [181];
  
void curPath(ros::Publisher &obsPub)
{
  look_ahead::Obstacles obsData;  //create an instance of the obstacle msg

  for (uint i = 0; i < sizeof(curLaserData); i++)
    {
      if(curLaserData[i] < cNotificationDistance)
	{
	  obsData.current.exists = 1;
	  obsData.current.distance = cNotificationDistance;
	  
	  obsPub.publish(obsData);

	}
      obsData.current.exists = 0;
      obsData.current.distance = 0.0;
      obsPub.publish(obsData);

    }
}


void nextPath(ros::Publisher &obsPub)
{
  look_ahead::Obstacles obsData;
  
  for(uint i = 0; i < sizeof(nextLaserData); i++)
    {
      if(nextLaserData[i] < cNotificationDistanceNext)
	{
	  obsData.next.exists = 1;
	  obsData.next.distance = cNotificationDistanceNext;
	}
      obsData.next.exists = 0;
      obsData.next.distance = 0.0;

    }
}




void laserCallback(const sensor_msgs::LaserScan::ConstPtr& laserScan)
{
  while(laserScan->range_max < 10)
    {
      for(uint i = 0; i < laserScan->ranges.size(); i++)
	{
	 curLaserData[i] = laserScan->ranges[i];
	}
    }
  
  for(int j = 10; j < 20; j++)
    {
      nextLaserData[j] = laserScan->ranges[j];
    }
}
	


int main(int argc, char **argv)
{
    ros::init(argc, argv, "laser_listener");

    ros::NodeHandle n;
    
    ros::Subscriber laserSub = n.subscribe("base_scan",1,laserCallback);
    ros::Publisher obsPub = n.advertise<look_ahead::Obstacles>("obstacles",1);
    
    ros::spin();

    return 0;
}

