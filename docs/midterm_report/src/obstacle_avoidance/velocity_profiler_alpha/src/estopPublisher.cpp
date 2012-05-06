#include <ros/ros.h>
#include <std_msgs/Bool.h>


int main(int argc, char **argv)
{
  ros::init(argc,argv,"estopPublisher"); // name of the node
  ros::NodeHandle n;
  
  ros::Publisher pub = n.advertise<std_msgs::Bool>("motors_enabled",1);
  
  while(!ros::Time::isValid()) {}

  ros::Rate naptime(10);

  ros::Duration onPeriod(5);
  ros::Duration offPeriod(1);
  ros::Time lastHit = ros::Time::now();
  std_msgs::Bool motors_enabled;
  motors_enabled.data = 1;
  while(ros::ok())
  {
    if(motors_enabled.data) 
    {
      if((ros::Time::now()-lastHit) > onPeriod)
      {
        motors_enabled.data = 0;
        lastHit = ros::Time::now();
      }
    }
    else
    {
      if((ros::Time::now() - lastHit) > offPeriod)
      {
	motors_enabled.data = 1;
	lastHit = ros::Time::now();
      }
    }

    pub.publish(motors_enabled);
    naptime.sleep();
  }
  return 0;
}
