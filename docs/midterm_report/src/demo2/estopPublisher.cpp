#include <ros/ros.h>
#include <std_msgs/Bool.h>

// simulates an estop
int main(int argc, char **argv)
{
  ros::init(argc,argv,"estopPublisher"); // name of the node
  ros::NodeHandle n;
  
  ros::Publisher pub = n.advertise<std_msgs::Bool>("motors_enabled",1);
  
  while(!ros::Time::isValid()) {}

  ros::Rate naptime(10); // refresh rate of 10 Hz

  ros::Duration onPeriod(5); // how long to enable estop
  ros::Duration offPeriod(1); // how long ti disable it
  ros::Time lastHit = ros::Time::now(); // toggle switch
  std_msgs::Bool motors_enabled;
  motors_enabled.data = 1;
  while(ros::ok()) // while not quit
  {
    if(motors_enabled.data)  // if the motors are already enabled
    {
      if((ros::Time::now()-lastHit) > onPeriod) // and its time to switch
      {
        motors_enabled.data = 0; // turn them off
        lastHit = ros::Time::now();
      }
    }
    else // the motors are already off
    {
      if((ros::Time::now() - lastHit) > offPeriod) // and its time to switch
      {
	motors_enabled.data = 1; // turn them on
	lastHit = ros::Time::now();
      }
    }

    pub.publish(motors_enabled);
    naptime.sleep();
  }
  return 0;
}
