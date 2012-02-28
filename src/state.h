/*
 * state.h
 *
 *  Created on: Feb 27, 2012
 *      Author: Devin
 */

#ifndef STATE_H_
#define STATE_H_

#include <eecs_376_alpha/PathSegment.h>
#include <tf/transform_datatypes.h>
//#include <nav_msgs/Odometry.h>
//#include <ros/ros.h>

class State
{
public:
	// constructors
	State();
	State(eecs_376_alpha::PathSegment segment);

	//methods
	void updateState(double vCmd, double dt);
	void newSegment(eecs_376_alpha::PathSegment segment);
	void stop();

	//getters
	double getXPath();
	double getYPath();
	double getPsiPath();

	double getVCmd(); // will return in m/s for arcs and straights and radians/s for spins
	double getSegDistDone(); // will return in m for arcs and straights and radians for spins

	eecs_376_alpha::PathSegment* getSegment();

private:
	//ros::Subscriber sub;
	double xPath,yPath,psiPath;
	double xAct,yAct,psiAct;
	double vCmd;
	double segDistDone;
	eecs_376_alpha::PathSegment *currSeg;

	//void poseCallback(const nav_msgs::Odometry::ConstPtr& pose);
};

#endif /* STATE_H_ */
