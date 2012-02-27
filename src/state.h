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

class State
{
public:
	// constructors
	State(double x0, double y0, double psi0);
	State(double x0, double y0, double psi0, eecs_376_alpha::PathSegment segment);

	//methods
	void updateState(double vCmd, double dt);
	void newSegment(eecs_376_alpha::PathSegment segment)
	void stop();

	//getters
	double getXPath();
	double getYPath();
	double getPsiPath();

	double getVCmd(); // will return in m/s for arcs and straights and radians/s for spins
	double getSegDistDone(); // will return in m for arcs and straights and radians for spins

	eecs_376_alpha::PathSegment getSegment();

private:
	double xPath,yPath,psiPath;
	double vCmd;
	double segDistDone;
	eecs_376_alpha::PathSegment currSeg;
};

#endif /* STATE_H_ */
