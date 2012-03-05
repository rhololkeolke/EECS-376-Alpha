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
#include <geometry_msgs/Vector3.h>

class State
{
public:
	// constructors
	State(); // setup the subscribers and initialize empty values
	State(double dtDes, geometry_msgs::Point initPos, double initPsi, eecs_376_alpha::PathSegment segment); // setup the subscribers and initialize the first segment

	//methods
	void updateState(geometry_msgs::Twist vel); // propagate along the path given with the last commands. Ignore actual robot position
	void updateState(geometry_msgs::Twist vel, geometry_msgs::Point pos, double psi); // propagate along the path given the last commands
	void newSegment(eecs_376_alpha::PathSegment *segment); // replace the segment with the next segment
	void stop();

	//getters
	geometry_msgs::Point getPathPoint();
	double getXPath();
	double getYPath();
	double getPsiPath();

	geometry_msgs::Point getActPoint();
	double getXAct();
	double getYAct();
	double getPsiAct();

	geometry_msgs::Twist State::getVelocity();
	double getVCmd(); // will return in m/s for arcs and straights
	double getOCmd(); /// will return radians/s
	double getSegDone(); // will return in m for arcs and straights and radians for spins. This is the distance along the path, not the distance moved

	eecs_376_alpha::PathSegment* getSegment();

private:
	// stores the time step
	double dt;

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

	//methods
	static geometry_msgs::Vector3 getVector(geometry_msgs::Point p0, geometry_msgs::Point p1); // calculate vector from 2 points
	static geometry_msgs::Vector3 unitVector(geometry_msgs::Vector3 r); // calculate a unit vector in the direction of r
	static double dotProduct(geometry_msgs::Vector3 r0, geometry_msgs::Vector3 r1);
};

#endif /* STATE_H_ */
