#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <eecs_376_alpha/PathSegment.h>

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc,argv,"velocityProfilerTest");
  ros::NodeHandle n;
  ros::Publisher segmentPub = n.advertise<eecs_376_alpha::PathSegment>("path_segs",1);

  ros::Rate naptime(10);

  while(!ros::Time::isValid()) {}

  eecs_376_alpha::PathSegment segment;

  int i = 0;
  while(ros::ok())
    {
      segment.seg_number = i;
      segment.max_speeds.linear.x = 5.0;
      segmentPub.publish(segment);
      i++;
      naptime.sleep();
    }

  return 0;
}
