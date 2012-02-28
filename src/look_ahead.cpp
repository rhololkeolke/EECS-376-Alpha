#include <ros/ros.h>
#include <cwru_base/Pose.h>
#include <sensor_msgs/LaserScan.h>
#include <eecs_376_alpha/Obstacles.h>
#include <iostream> 



using namespace std;

const uint cPings = 181;
const double cNotificationDistance = 0.75; //distance at which to publish that an obstacle exists for current path

double curLaserData [cPings]; //current path lidar info

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
  ros::Rate loop_rate(10);   

  eecs_376_alpha::Obstacles obsData;  //create an instance of the obstacle msg
  
  for (uint i = 0; i < cPings;  i++)
    {
      if(curLaserData[i] < cNotificationDistance)
	{
 	  obsData.exists = true; //boolean for obstacle existance
	  obsData.distance = 0;
	  obsPub.publish(obsData);
	}

      obsData.exists = false;
      obsData.distance = 0.0;
      obsPub.publish(obsData);
    }
}

void setFalse(ros::Publisher &obsPub)
{
  ros::Time time = ros::Time::now();


  eecs_376_alpha::Obstacles obsData;
  
  obsData.exists = false;
  obsData.distance = 0.0;
  obsPub.publish(obsData);

}
  



int main(int argc, char **argv)
{
  ros::init(argc, argv, "laser_listener");

  ros::NodeHandle n;
    
  ros::Subscriber laserSub = n.subscribe("base_scan",1,laserCallback);
  ros::Publisher obsPub = n.advertise<eecs_376_alpha::Obstacles>("obstacles",1);

  while(!ros::Time::isValid()) {}

  ros::Rate naptime(100);  
  while(ros::ok())
    {
      curPath(obsPub);

      ros::spinOnce();

      naptime.sleep();
    }
  

    


  return 0;
}
