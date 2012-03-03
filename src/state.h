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
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>

class State
{
public:
	// constructors
	State(); // setup the subscribers and initialize empty values
	State(eecs_376_alpha::PathSegment segment); // setup the subscribers and initialize the first segment

	//methods
	void updateState(double v_cmd, double o_cmd, double dt); // propagate along the path given the last commands
	void newSegment(eecs_376_alpha::PathSegment segment);
	void stop();

	//getters
	double getXPath();
	double getYPath();
	double getPsiPath();

	double getVCmd(); // will return in m/s for arcs and straights and radians/s for spins
	double getOCmd();
	double getSegDone(); // will return in m for arcs and straights and radians for spins

	eecs_376_alpha::PathSegment* getSegment();

private:
	// stores the x,y, and heading of the desired path
	geometry_msgs::Point pathPos;
	double pathPsi;

	// stores the x,y, and heading of the actual robot
	geometry_msgs::Point actPos;
	double actPsi;

	// stores the linear and angular velocity commands
	geometry_msgs::Twist velocity;

	// stores the distance along the path completed
	double segDone;

	// stores the segment currently being propagated
	eecs_376_alpha::PathSegment *currSeg;
};

#endif /* STATE_H_ */
