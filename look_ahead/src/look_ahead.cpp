 #include <ros/ros.h>
 #include <cwru_base/Pose.h>
 #include <sensor_msgs/LaserScan.h>
 #include<geometry_msgs/Twist.h>
 #include<tf/transform_datatypes.h>
 #include<look_ahead/Obstacles.h>
 #include<look_ahead/PathSegment.h>
 #include <iostream> 

using namespace std;

const uint cPings = 181;
const double cNotDist = 0.75; //distance at which to publish that an obstacle exists for current path
double curLaserData [cPings]; //current path lidar info

double yaw;
double seg_length;
 double seg_type;
double seg_number;
//Summary: Receive information about the current path and place them into globals
//Parameters: Path Segement Data
//Return: Nada
 //Todo:
void segCallback(const look_ahead::PathSegment::ConstPtr& pathSeg)
 {
   seg_length = pathSeg->seg_length;
   yaw= tf::getYaw(pathSeg->init_tan_angle);
   seg_type = pathSeg-> seg_type;
   seg_number = pathSeg-> seg_number;
 }


//Summary: Receive scan for the current path and place range data into an array
//Parameters: Laser Scan Data
//Return: Nothing
//Todo: Determine if scan ranges are appropriate
void laserCallback(const sensor_msgs::LaserScan::ConstPtr& laserScan)
 {    
   for(uint i = 0; i < laserScan->ranges.size(); i++)
     {
       curLaserData[i] = laserScan->ranges[i];
     }
 }  



//Summary: Publishes information regarding whether or not there is an obstalce within the current path segement
//Parameters: The Obstacle Publisher
//Return: Nothing
//Todo: Implement SegStatus msg type, Determine why Obstacles topic only publishes sometimes
void curPath(ros::Publisher &obsPub)
{
   look_ahead::Obstacles obsData;  //create an instance of the obstacle msg
   ros::Time time = ros::Time::now();
   ros::Rate loop_rate(10);   
   
     for (uint i = 0; i < cPings;  i++)
       { 
	 if(seg_type == 1) 
	   {
	     if(curLaserData[i] < cNotDist)
	       {
		 obsData.exists = true;
		 obsPub.publish(obsData);
	       }
	     obsData.exists = false;
	     obsPub.publish(obsData);
	   }
	 else if(seg_type == 2)
	   {
	     
	   }
	 
	 else if(seg_type ==3)
	   {
	   }
       }
     loop_rate.sleep();
 }

void laser(ros::Publisher &laserPub)
{
  sensor_msgs::LaserScan scan;
  
  scan.range_min = 0;
  scan.range_max = 1;
  laserPub.publish(scan);
}  






int main(int argc, char **argv)
{
    ros::init(argc, argv, "look_ahead");

    ros::NodeHandle n;
    
    ros::Subscriber laserSub = n.subscribe("base_scan",1,laserCallback);
    ros::Subscriber sub=n.subscribe("segTopic",1,segCallback);
    ros::Publisher obsPub = n.advertise<look_ahead::Obstacles>("obstacles",1);

    while(!ros::Time::isValid()) {}

      
    curPath(obsPub);
    
    ros::spinOnce();

    return 0;
}

