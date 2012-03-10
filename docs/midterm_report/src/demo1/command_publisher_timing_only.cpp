#include <ros/ros.h>
#include <geometry_msgs/Twist.h> // pre-defined data type for velocities

#define PI 3.14159 // set the number of decimals of PI up here
#define RATE 10 // set the rate. Use 10 for simulator and 20 for robot

// Passing the publisher by reference eliminates the need to
// reinitialize it every function call.
// If distance is negative then the robot will move backwards
void straight(ros::Publisher& pub, double distance)
{
  geometry_msgs::Twist vel_object; // data type for velocities

  double runtime; // time to run for
  if(distance <0.0) // check if backwards or forwards
  {
    runtime = distance*-1.0; // this is assuming 1 m/s
  }
  else
  {
    runtime = distance; // this is assuming 1 m/s
  }
  ros::Duration run_duration(runtime); // ros type for storing durations

  ros::Duration elapsed_time; // keeps track of how long the robot has
                              // been moving
  ros::Rate naptime(RATE); // the loop rate of the node in HZ

  ros::Time birthday = ros::Time::now(); // initialize the start time
  ROS_INFO("Started a straight segment at %f for %f seconds", birthday.toSec(), run_duration.toSec());
  while(ros::ok() && elapsed_time < run_duration) // while no quit
  {                                               // command and still
                                                  // more to go 
    elapsed_time = ros::Time::now()-birthday;
    if(distance > 0) // check if backwards or forwards
    {
      vel_object.linear.x = 1.0; // positive for forwards
    }
    else
    {
      vel_object.linear.x = -1.0; // negative for backwards
    }

    vel_object.angular.z = 0.0; // no turn
    pub.publish(vel_object); // publish the value to the robot motors
    naptime.sleep(); // wait so that the loop rate is maintained
  }
  // before leaving publish a 0.0 to guarantee the robot is completely stopped
  vel_object.linear.x = 0.0;
  vel_object.angular.z = 0.0;
  pub.publish(vel_object);
}

void turn(ros::Publisher& pub, double angle)
{
  //Pretty much the same code as straight only angles instead of distances
  geometry_msgs::Twist vel_object;

  double runtime;
  if(angle < 0.0)
  {
    runtime = angle * -1.0;
  }
  else
  {
    runtime = angle;
  }
  ros::Duration run_duration(runtime);

  ros::Duration elapsed_time;
  ros::Rate naptime(RATE);
  
  ros::Time birthday = ros::Time::now();
  ROS_INFO("Started a turn segment at %f for %f seconds", birthday.toSec(),run_duration.toSec());
  while(ros::ok() && elapsed_time < run_duration)
  {
    elapsed_time = ros::Time::now()-birthday;
    vel_object.linear.x = 0.0;

    if(angle > 0)
    {
      vel_object.angular.z = 1.0;
    }
    else
    {
      vel_object.angular.z = -1.0;
    }

    pub.publish(vel_object);
    naptime.sleep();
  }

  vel_object.linear.x = 0.0;
  vel_object.angular.z = 0.0;
  pub.publish(vel_object); 
}

int main(int argc, char **argv)
{
  ros::init(argc,argv,"command_publisher_timing_only"); // name of this node
  ros::NodeHandle n;

  // set the node up to publish the motor commands
  ros::Publisher pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1);

  while(!ros::Time::isValid()) {} // wait for everything to be up and running

  // desired path segments
  // each passes the publish by reference and a desired distance/angle
  straight(pub,3.9);
  turn(pub,-PI/2+0.2);
  straight(pub,12.0);
  turn(pub,-PI/2+.1);
  straight(pub,4.0);
 
  return 0;
}
	
