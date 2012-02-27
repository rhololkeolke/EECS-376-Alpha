#include <ros/ros.h>
#include <geometry_msgs/Twist.h> // pre-defined data type for velocities
#include <math.h>
#include <std_msgs/Bool.h>
#include <cwru_base/cRIOSensors.h>
#include "state.h"

#define PI 3.14159 // set the number of decimals of PI up here
#define RATE 20.0 // set the rate of refresh

bool stopped = false; // stores the value of the estop

void estopCallback(const std_msgs::Bool::ConstPtr& estop)
{
  stopped = !(estop->data);
}
    
void straight(ros::Publisher& pub, double distance)
{

  geometry_msgs::Twist vel_object;

  ros::Rate naptime(RATE);

  double dt = 1/RATE;
  double v_max; // the maximum speed in m/s
  double a_max; // the maximum accelration in m/s^2
  double segLength;
  if(distance < 0.0){ // this function assumes the the sign of distance determines the direction
    v_max = -1.0;
    a_max = -0.25;
    segLength = -1*distance;
  }
  else{
    v_max = 1.0;
    a_max = 0.25;
    segLength = distance;
  }
  
  double segDistDone = 0.0; // distance covered so far along current path segment
  
  ros::Duration T_accel(v_max/a_max); // assume that the robot is starting from rest
  ros::Duration T_decel(T_accel); // for a trapezoid the decel and the accel should be the same
  double dist_accel = 0.5*fabs(a_max)*pow(T_accel.toSec(),2);
  double dist_decel = dist_accel; // same as the ramp-up distance
  double dist_const_v = segLength - dist_accel - dist_decel; // if this is <0, then the robot will never reach full speed
  ros::Duration T_const_v(dist_const_v/fabs(v_max)); // will be <0 if the robot never reachs full speed
  ros::Duration T_segment_tot = T_accel + T_decel + T_const_v;

  // initialize the State for that segment
  State currentState = State();

  // these will store the values for the command
  double v_cmd = 0.0;
  double omega_cmd = 0.0; // this should never change in the case of a straight line
  double v_scheduled = 0.0;

  if(a_max < 0.0)
    ROS_INFO("Started a stright line segment for %f meters in the negative x direction", segLength);
  else
    ROS_INFO("Started a straight line segment for %f meters in the positive x direction", segLength);

  bool lastStopped = stopped; // keeps track of the last state of stopped

  while(segDistDone < segLength && ros::ok()) {
    
    if(stopped) // stopped is updated by the estopCallback function asynchronously
    {
      lastStopped = stopped;
      ROS_INFO("STOPPED!");
      v_cmd = 0; // we don't want the robot to move
      omega_cmd = 0;
      currentState.stop(); // set the internal state to no velocity

      vel_object.linear.x = 0.0; // this is so the simulator acts correctly when using our estopPublisher program
      vel_object.angular.z = 0.0;
     
      pub.publish(vel_object);
      naptime.sleep(); // this is here so that the loop keeps the same rate and doesn't take up all the CPU time
      continue; // restart the while loop
    }
    else if(lastStopped) // the last iteration was stopped but this one isn't
    {
      ROS_INFO("Sleeping for 2.0 seconds");
      lastStopped = 0; // set lastStopped to false
      ros::Duration(2.0).sleep(); // this is so the motor controllers have time to come back online
    }
      
    currentState.updateState(v_cmd, omega_cmd, dt); // advance where the robot thinks its at

    // extract all of that information from the State object
    segDistDone = currentState.getDistDone();
    v_cmd = currentState.getVCmd();
    if(segDistDone<dist_accel){
      if(a_max < 0.0) { // don't want negatives inside the sqrt
	v_scheduled = -1*sqrt(2*segDistDone*fabs(a_max));
      } else {
	v_scheduled = sqrt(2*segDistDone*a_max);
      }
      if(fabs(v_scheduled) < fabs(a_max)*dt) {
	v_scheduled = a_max*dt; // add some extra to avoid sticking in place
      }
    }
    else if(segDistDone<dist_accel+dist_const_v) { // const velocity phase (may not always occur)
      v_scheduled = v_max;
    }
    else {
      if(a_max < 0.0) { // don't want negatives inside the sqrt
	v_scheduled = -1*sqrt(2*(segLength-segDistDone)*-1*a_max);
      }
      else {
	v_scheduled = sqrt(2*(segLength-segDistDone)*a_max);
      }
    }

    // check scheduled velocity against most recent velocity
    if(fabs(v_cmd) < fabs(v_scheduled)) {
     double v_test = v_cmd + a_max*dt;
     if(fabs(v_test) < fabs(v_scheduled)) {
       v_cmd = v_test;
     } else {
       v_cmd = v_scheduled;
     }
    }
    else if(fabs(v_cmd) > fabs(v_scheduled)) {
      double v_test = v_cmd - ( 1.2*a_max*dt);
      if(fabs(v_test) > fabs(v_scheduled)) {
	v_cmd = v_test;
      } else {
	v_cmd = v_scheduled;
      }
    }

    vel_object.linear.x = v_cmd;
    vel_object.angular.z = 0.0;
    pub.publish(vel_object);

    naptime.sleep();
  }
  vel_object.linear.x = 0.0;
  vel_object.angular.z = 0.0;
  pub.publish(vel_object);
}
     
void turn(ros::Publisher& pub, double angle)
{
 
  geometry_msgs::Twist vel_object;

  ros::Rate naptime(RATE);

  double dt = 1/RATE;
  double o_max; // the maximum angular velocity in m/s
  double a_max; // the maximum angular acceleration
  double segRads; // radians for this segment
  if(angle < 0.0) { // the sign of the angle determines the direction
    o_max = -1.0;
    a_max = -0.5;
    segRads = -1*angle; // radians in the segment should always be positive so that the logic below works
  }
  else {
    o_max = 1.0;
    a_max = 0.5;
    segRads = angle;
  }

  double segRadsDone = 0.0; // radians covered so far along current rotation
  
  ros::Duration T_accel(o_max/a_max); // assume the robot is starting from rest
  ros::Duration T_decel(T_accel); // for trapezoidal profiling the decel and the accel should be the same
  double rad_accel = 0.5*fabs(a_max)*pow(T_accel.toSec(),2);
  double rad_decel = rad_accel; // same as the ramp up radians
  double rad_const_o = segRads - rad_accel - rad_decel; // if this is <0, then robot will never read full speed
  ros::Duration T_const_v(rad_const_o/fabs(o_max)); // will be <0 if the robot never reachs full speed
  ros::Duration T_segment_tot = T_accel + T_decel + T_const_v;

  // initialize the State for this segment
  State currentState = State();

  // these will store the values for the command
  double v_cmd = 0.0; // this should never change when the robot is spinning in place
  double o_cmd = 0.0;
  double o_scheduled = 0.0;

  if(a_max < 0.0)
    ROS_INFO("Started turning for %f radians, in the negative direction about the z axis", segRads);
  else
    ROS_INFO("Started turning for %f radians, in the positive direction about the z axis", segRads);

  bool lastStopped = stopped;

  while(segRadsDone < segRads && ros::ok()) {

    if(stopped) // see straight for how this works
    {
      lastStopped = stopped;
      ROS_INFO("STOPPED!");
      currentState.stop();

      v_cmd = 0;
      o_cmd = 0;

      vel_object.linear.x = 0.0;
      vel_object.angular.z = 0.0;

      pub.publish(vel_object);
      naptime.sleep();
      continue;
    }  
    else if(lastStopped)
    {
      ROS_INFO("Sleeping for 2.0 seconds");
      lastStopped = 0;
      ros::Duration(2.0).sleep();
    }
    
    currentState.updateState(v_cmd, o_cmd, dt); // advance where the robot thinks its at

    // extract all of that information from the State object
    segRadsDone = currentState.getAngDone();
    o_cmd = currentState.getOCmd();
    if(segRadsDone < rad_accel) {
      if(a_max < 0.0) { // don't want negatives inside the sqrt
	o_scheduled = -1*sqrt(2*segRadsDone*fabs(a_max));
      }
      else {
	o_scheduled = sqrt(2*segRadsDone*a_max);
      }

      if(fabs(o_scheduled) < fabs(a_max)*dt) {
	o_scheduled = a_max*dt; // add some extra to avoid sticking in place
      }
    }
    else if(segRadsDone<rad_accel+rad_const_o) { // const angular velocity phase (may not always occur)
      o_scheduled = o_max;
    }
    else {
      if(o_max < 0.0) {
	o_scheduled = -1*sqrt(2*(segRads-segRadsDone)*fabs(a_max));
      }
      else {
	o_scheduled = sqrt(2*(segRads-segRadsDone)*a_max);
      }
    }

    // check scheduled angular velocity against most recent angular velocity
    if(fabs(o_cmd) < fabs(o_scheduled)) {
      double o_test = o_cmd + a_max*dt;
      if(fabs(o_test) < fabs(o_scheduled)) {
	o_cmd = o_test;
      }
      else {
	o_cmd = o_scheduled;
      }
    }
    else if(fabs(o_cmd) > fabs(o_scheduled)) {
      double o_test = o_cmd - (1.2*a_max*dt);
      if(fabs(o_test) > fabs(o_scheduled)) {
	o_cmd = o_test;
      }
      else {
	o_cmd = o_scheduled;
      }
    }

    vel_object.linear.x = 0.0;
    vel_object.angular.z = o_cmd;
    pub.publish(vel_object);
      
    naptime.sleep(); // wait until its time to publish.  This keeps publisher at rate specified by RATE
  }

  vel_object.linear.x = 0.0;
  vel_object.angular.z = 0.0;
  pub.publish(vel_object);
}

int main(int argc, char **argv)
{
  ros::init(argc,argv,"velocity_profiler"); // name of this node
  ros::NodeHandle n;

  ros::Publisher pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1);
  ros::Subscriber sub = n.subscribe("motors_enabled",1,estopCallback); // listen for estop values

  // this is necessary or callbacks will never be processed.
  // AsyncSpinner lets them run in the background
  ros::AsyncSpinner spinner(0); // use same number of threads as cores
  spinner.start();
 
  while(!ros::Time::isValid()) {} // wait for simulator

  // this is the set of hard coded directions that will make the robot drive to the vending
  // machines
  straight(pub,4.2);
  turn(pub,-PI/2);
  straight(pub,12.5);
  turn(pub,-PI/2);
  straight(pub,4.0);

  return 0;
}
	
