#include <ros/ros.h>
#include <geometry_msgs/Twist.h> // pre-defined data type for velocities
#include <math.h>
#include <std_msgs/Bool.h>
#include <cwru_base/cRIOSensors.h>
#include <velocity_profiler/PathSegment.h>
#include <velocity_profiler/SegStatus.h>
#include <velocity_profiler/Obstacles.h>
#include <iostream>
#include <tf/transform_datatypes.h>
#include "lockedQueue.h"

using namespace std;

const double PI = 3.14159; // set the number of decimals of PI up here
const double RATE = 20.0; // set the rate of refresh

bool stopped = false; // stores the value of the estop

// stores the values of the obstacles in the way
bool obs = false;
double obs_dist = 0.0;

double lastVCmd = 0;
double lastOCmd = 0;

int seg_number = 0;

lockedQueue<velocity_profiler::PathSegment*> segments;

class State
{
public:
  State();
  void updateState(double v_cmd, double omega_cmd, double dt);
  void stop();
  // getters
  double getX();
  double getY();
  double getPhi();
  double getVCmd();
  double getOCmd();
  double getDistDone();
  double getAngDone();
private:
  double x,y,phi,v,omega,segDistDone,spinAngDone;
};

State::State() {
  x = 0.0;
  y = 0.0;
  phi = 0.0;
  v = 0.0; // assume start from rest
  omega = 0.0; // assume start from rest
  segDistDone = 0.0;
  spinAngDone = 0.0;
};

void State::updateState(double v_cmd, double omega_cmd, double dt) {
  double avg_v = (v+v_cmd)/2; // average the velocity over this time step
  double avg_omega = (omega+omega_cmd)/2; // average angular velocity over this time step
  double avg_phi = phi + avg_omega*dt/2; // average heading over time step
  
  x = x+avg_v*dt*cos(avg_phi); // advance x coordinate
  y = y+avg_v*dt*sin(avg_phi); // advance y coordinate
  phi = phi + avg_omega*dt; // advance the heading
  segDistDone = segDistDone+fabs(avg_v)*dt; // update the distance traveled
  spinAngDone = spinAngDone+fabs(avg_omega)*dt; // update the sin-in-place angle done

  v = v_cmd;
  omega = omega_cmd;
}

void State::stop()
{
  v = 0.0;
  omega = 0.0;
}

double State::getX() {
  return x;
}

double State::getY() {
  return y;
}

double State::getPhi() {
  return phi;
}

double State::getVCmd() {
  return v;
}

double State::getOCmd() {
  return omega;
}

double State::getDistDone() {
  return segDistDone;
}

double State::getAngDone() {
  return spinAngDone;
}

void estopCallback(const std_msgs::Bool::ConstPtr& estop)
{
  stopped = !(estop->data);
}

void obstaclesCallback(const velocity_profiler::Obstacles::ConstPtr& obsData)
{
  obs = obsData->exists;
  obs_dist = obsData->distance;
}

void pathSegCallback(const velocity_profiler::PathSegment::ConstPtr& seg)
{
  velocity_profiler::PathSegment *newSeg = new velocity_profiler::PathSegment();
  newSeg->seg_number = seg->seg_number;
  newSeg->seg_type = seg->seg_type;
  newSeg->curvature = seg->curvature;
  newSeg->seg_length = seg->seg_length;
  newSeg->ref_point = seg->ref_point;
  newSeg->init_tan_angle = seg->init_tan_angle;
  newSeg->max_speeds = seg->max_speeds;
  newSeg->accel_limit = seg->accel_limit;
  newSeg->decel_limit = seg->decel_limit;
  segments.push(newSeg);
}

void velCallback(const geometry_msgs::Twist::ConstPtr& vel)
{
  lastVCmd = vel->linear.x;
  lastOCmd = vel->angular.z;
}
    
void straight(ros::Publisher& pub, ros::Publisher& segStatusPub, double distance)
{

  geometry_msgs::Twist vel_object;

  ros::Rate naptime(RATE);

  double dt = 1/RATE;
  double v_max; // the maximum speed in m/s
  double a_max; // the maximum accelration in m/s^2
  double segLength;
  if(distance < 0.0){ // this function assumes the the sign of distance determines the direction
    v_max = -0.5;
    a_max = -0.25;
    segLength = -1*distance;
  }
  else{
    v_max = 0.5;
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
  double v_cmd = lastVCmd;
  double omega_cmd = lastOCmd; // this should never change in the case of a straight line
  double v_scheduled = 0.0;

  if(a_max < 0.0)
    ROS_INFO("Started a stright line segment for %f meters in the negative x direction", segLength);
  else
    ROS_INFO("Started a straight line segment for %f meters in the positive x direction", segLength);

  bool lastStopped = stopped; // keeps track of the last state of stopped
  bool lastobs = false;
  double startingPos = 0.0;
  double obsDist = 0.0;
  double decel_rate = 0.0;

  while(segDistDone < segLength && ros::ok()) {
    
    v_cmd = lastVCmd;
    omega_cmd = lastOCmd;
    if(stopped) // stopped is updated by the estopCallback function asynchronously
    {
      lastStopped = stopped;
      ROS_INFO("STOPPED!");
      v_cmd = 0; // we don't want the robot to move
      omega_cmd = 0;
      currentState.stop(); // set the internal state to no velocity

      velocity_profiler::SegStatus status;
      status.segComplete = false;
      status.seg_number = seg_number;
      status.progress_made = currentState.getDistDone();

      segStatusPub.publish(status);      


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

    if(obs && !lastobs)
    {
      //ROS_INFO("Initializing obstacles");
      startingPos = currentState.getDistDone();
      obsDist = obs_dist;
      lastobs = true;
      decel_rate = pow(v_cmd,2)/(obs_dist-.1);
      cout << "obs_dist: " << obs_dist << ", decel_rate: " << decel_rate << ", startingPos: " << startingPos << endl; 
    }

    if(lastobs && obs && fabs(distance)-startingPos >= obs_dist) // there is an obstacle in the way
    {
      //ROS_INFO("Running obstacle code");
      // calculate the deceleration rate
            
      if(v_cmd > .001)
      {
	v_cmd = v_cmd - decel_rate*dt;
	currentState.updateState(v_cmd, omega_cmd, dt);
	vel_object.linear.x = v_cmd;
      }
      else
      {
	vel_object.linear.x = 0.0;
	currentState.stop();
      }

      velocity_profiler::SegStatus status;
      status.segComplete = false;
      status.seg_number = seg_number;
      status.progress_made = currentState.getDistDone();

      segStatusPub.publish(status);      

      vel_object.angular.z = 0.0;
      pub.publish(vel_object);

      naptime.sleep();
      continue;
    }
    else
    {
      lastobs = false;
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

    velocity_profiler::SegStatus status;
    status.segComplete = false;
    status.seg_number = seg_number;
    status.progress_made = currentState.getDistDone();

    segStatusPub.publish(status);

    vel_object.linear.x = v_cmd;
    vel_object.angular.z = 0.0;
    pub.publish(vel_object);

    naptime.sleep();
  }
  velocity_profiler::SegStatus status;
  status.segComplete = true;
  status.seg_number = seg_number;
  status.progress_made = currentState.getDistDone();

  segStatusPub.publish(status);

  vel_object.linear.x = 0.0;
  vel_object.angular.z = 0.0;
  pub.publish(vel_object);
}
     
void turn(ros::Publisher& pub, ros::Publisher& segStatusPub, double angle)
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
    v_cmd = lastVCmd;
    o_cmd = lastOCmd;

    if(stopped) // see straight for how this works
    {
      lastStopped = stopped;
      ROS_INFO("STOPPED!");
      currentState.stop();

      v_cmd = 0;
      o_cmd = 0;

      vel_object.linear.x = 0.0;
      vel_object.angular.z = 0.0;

  velocity_profiler::SegStatus status;
  status.segComplete = false;
  status.seg_number = seg_number;
  status.progress_made = currentState.getDistDone();

  segStatusPub.publish(status);


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

  velocity_profiler::SegStatus status;
  status.segComplete = false;
  status.seg_number = seg_number;
  status.progress_made = currentState.getDistDone();

  segStatusPub.publish(status);

    vel_object.linear.x = 0.0;
    vel_object.angular.z = o_cmd;
    pub.publish(vel_object);
      
    naptime.sleep(); // wait until its time to publish.  This keeps publisher at rate specified by RATE
  }
  velocity_profiler::SegStatus status;
  status.segComplete = true;
  status.seg_number = seg_number;
  status.progress_made = currentState.getDistDone();

  segStatusPub.publish(status);


  vel_object.linear.x = 0.0;
  vel_object.angular.z = 0.0;
  pub.publish(vel_object);
}

int main(int argc, char **argv)
{
  ros::init(argc,argv,"velocity_profiler"); // name of this node
  ros::NodeHandle n;

  ros::Publisher desVelPub = n.advertise<geometry_msgs::Twist>("des_vel",1);
  ros::Publisher segStatusPub = n.advertise<velocity_profiler::SegStatus>("seg_status",1);
  ros::Subscriber estopSub = n.subscribe("motors_enabled",1,estopCallback); // listen for estop values
  ros::Subscriber obsSub = n.subscribe("obstacles",1,obstaclesCallback);
  ros::Subscriber velSub = n.subscribe("cmd_vel",1,velCallback);
  ros::Subscriber pathSub = n.subscribe("path_seg",10,pathSegCallback);

  ros::Rate naptime(RATE);

  // this is necessary or callbacks will never be processed.
  // AsyncSpinner lets them run in the background
  ros::AsyncSpinner spinner(0); // use same number of threads as cores
  spinner.start();
 
  while(!ros::Time::isValid()) {} // wait for simulator

  velocity_profiler::PathSegment* currSeg = NULL;

  double dist = 0.0;
  while(ros::ok())
  {
    if(currSeg == NULL); // while there is nothing to do
    {
      if(segments.size() > 0) // see if something new was added to the queue
      {
	currSeg = segments.front(); // if so look at it
	segments.pop();

	if(currSeg->seg_type == 1)
	{
	  double xs = currSeg->ref_point.x;
	  double ys = currSeg->ref_point.y;
	
	  double desired_heading = tf::getYaw(currSeg->init_tan_angle);

	  double xf = xs + currSeg->seg_length*cos(desired_heading);
	  double yf = ys + currSeg->seg_length*sin(desired_heading);

	  dist = sqrt(pow(xf-xs,2)+pow(yf-ys,2));
	  seg_number = currSeg->seg_number;
	  straight(desVelPub,segStatusPub,dist);
	  delete currSeg;
	  currSeg = NULL;
	}
	else if(currSeg->seg_type == 3)
	{
	  seg_number = currSeg->seg_number;
	  dist = currSeg->seg_length;	  
	  turn(desVelPub,segStatusPub,dist);
	  delete currSeg;
	  currSeg = NULL;
	}
	else
	{
	  delete currSeg;
	  currSeg = NULL;
	}
      }
      else
      {
	geometry_msgs::Twist vel_object;
	vel_object.linear.x = 0.0;
	vel_object.angular.z = 0.0;
	desVelPub.publish(vel_object);
	naptime.sleep();
	continue; // nothing to do start again
      }
    }
  }
  // this is the set of hard coded directions that will make the robot drive to the vending
  // machines
  /*  straight(pub,4.2);
  turn(pub,-PI/2);
  straight(pub,12.5);
  turn(pub,-PI/2);
  straight(pub,4.0);*/

  return 0;
}
	
