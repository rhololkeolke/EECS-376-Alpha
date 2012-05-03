#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d_publisher.h>
#include <string>

int main(int argc, char** argv){

	string arg1 = string(argv[1]);
	//initialize ROS
	ros::init(argc, argv, arg1);

	//create a Transform Listener
	tf::TransformListener tf(ros::Duration(10));

	//we'll need a node handle
	ros::NodeHandle n;

	//create a ROS wrapper for the costmap, passing it a name and a reference to a TransformListener
	//which will configure itself based on parameters
	costmap_2d::Costmap2DROS costmap_ros(arg1, tf);

	//start processing data
	ros::spin();

	return(0);

}
