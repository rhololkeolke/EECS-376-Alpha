#include <ros/ros.h>
#include <cwru_base/Pose.h>
#include <sensor_msgs/LaserScan.h>
#include<laser_listener/Obstacles.h>
#include <iostream>


using namespace std;

const int cNotificationDistance = 0.75; //distance at which to publish that an obstacle exists
double laserData [181]; //lidar data for path is copied into this array 
  
void curPath(ros::Publisher &obsPub)
{
  laser_listener::Obstacles obsData;  //create an instance of the obstacle msg

  for (uint i = 0; i < sizeof(laserData); i++)
    {
      if(laserData[i] < cNotificationDistance)
	{
	  obsData.exists = 1;
	  obsData.distance = cNotificationDistance;
	  
	  obsPub.publish(obsData);

	}
      obsData.exists = 0;
      obsData.distance = cNotificationDistance;
      obsPub.publish(obsData);

    }
}


void nextPath()
{

}



void laserCallback(const sensor_msgs::LaserScan::ConstPtr& laserScan)
{
    for(uint i = 0; i < laserScan->ranges.size(); i++)
    {
      laserData[i] = laserScan->ranges[i];
    }
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "laser_listener");

    ros::NodeHandle n;
    
    ros::Subscriber laserSub = n.subscribe("base_scan",1,laserCallback);
    ros::Publisher obsPub = n.advertise<laser_listener::obstacles>("obstacles",1);
    
    ros::spin();

    return 0;
}

