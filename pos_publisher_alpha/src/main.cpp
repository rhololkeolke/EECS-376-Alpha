#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

using namespace std;

nav_msgs::Odometry last_odom;
geometry_msgs::PoseStamped last_map_pose;

geometry_msgs::PoseStamped temp;

tf::TransformListener *tfl;

void odomCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
  last_odom = *odom;
  temp.pose = last_odom.pose.pose;
  temp.header = last_odom.header;

  try
  {
    tfl->transformPose("map", temp, last_map_pose);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc,argv,"pos_publisher_alpha_main");
  ros::NodeHandle n;
  tfl = new tf::TransformListener();
  ros::Subscriber odomSub = n.subscribe<nav_msgs::Odometry>("odom",1,odomCallback);
  ros::Publisher posPub = n.advertise<geometry_msgs::PoseStamped>("map_pos",1);

  while(!ros::Time::isValid()) {}

  while(!tfl->canTransform("map","odom", ros::Time::now())) {}
  
  ros::Rate naptime(10);
  
  while(ros::ok())
  {
    ros::spinOnce();

    posPub.publish(last_map_pose);

    naptime.sleep();
  }

  delete tfl;
  return 0;
}
