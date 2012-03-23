#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <eecs_376_alpha/PathSegment.h>
#include <tf/transform_datatypes.h>

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc,argv,"velocityProfilerTest");
  ros::NodeHandle n;
  ros::Publisher segmentPub = n.advertise<eecs_376_alpha::PathSegment>("path_segs",1);

  ros::Rate naptime(10);

  while(!ros::Time::isValid()) {}

  eecs_376_alpha::PathSegment segment;

  for(int i = 0; i<10; i++)
    {
      segment.seg_number = i;
      segment.seg_type = 1;
      segment.max_speeds.linear.x = 1.0;
      segment.max_speeds.angular.z = 0.0;
      segment.accel_limit = 0.5;
      segment.decel_limit = -0.5;
      segment.init_tan_angle = tf::createQuaternionMsgFromYaw(0.0);
      segment.curvature = 0.0;
      segment.seg_length = 1.0;
      segment.ref_point.x = 0.0;
      segment.ref_point.y = 0.0;

      segmentPub.publish(segment);
      naptime.sleep();
    }

  return 0;
}
