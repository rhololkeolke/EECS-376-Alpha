/*
 * state.cpp
 *
 *  Created on: Feb 27, 2012
 *      Author: Devin
 */

#include "state.h"
#include <math.h>

const double PI = 3.14159;

State::State(double x0, double y0, double psi0)
{
	xPath = x0;
	yPath = y0;
	psiPath = psi0;
	currSeg = NULL;
}

State::State(double x0, double y0, double psi0, eecs_376_alpha::PathSegment segment)
{
	xPath = x0;
	yPath = y0;
	psiPath = psi0;
	currSeg = segment; // may make this pass by reference so that velocity profiler can update the ending positions and velocities via callbacks
}

void State::updateState(double vCmd, double dt)
{
	double dL = vCmd*dt;
	double expSegDistDone = segDistDone + dL; // compute how much more the robot should complete based on previous segment distance done

	if(currSeg.seg_type == 1) // straight line segment
	{
		// compute the theoretical line position
		double psiDes = tf::getYaw(currSeg.init_tan_angle);
		double rhoDes = 0; // this is zero because lines have no curvature
		// line starts at xPath and yPath and is an an angle psiDes
		double xDes = xPath+expSegDistDone*cos(psiDes);
		double yDes = yPath+expSegDistDone*sin(psiDes);
		// calculate the actual line position

		// project actual onto desired

		// calculate the segDistDone
	}
	else if(currSeg.seg_type == 2) // arc segment
	{
		double rhoDes = currSeg.curvature;
		double r = 1/fabs(rhoDes); // turn radius of inverse curvature
		double arcAngleStart;

		if(rhoDes >= 0)
			arcAngleStart = tf::getYaw(currSeg.init_tan_angle)-PI/2;
		else
			arcAngleStart = tf::getYaw(currSeg.init_tan_angle)+PI/2;

		double dAng = expSegDistDone*rhoDes;
		double arcAng = arcAngleStart+dAng;
		xDes = xPath + r*cos(arcAng);
		yDes = yPath + r*sin(arcAng);
		psiDes = tf::getYaw(currSeg.init_tan_angle)+dAng;
	}
	else if(currSeg.seg_type == 3)
	{
		xDes = xPath;
		yDes = yPath;
		psiDes = tf::getYaw(currSeg.init_tan_angle) + currSeg.curvature()*expSegDistDone;
	}
}

// make segment the new current segment
void State::newSegment(eecs_376_alpha::PathSegment segment)
{
	currSeg = segment;
}

void State::stop()
{
	vCmd = 0.0;
}

double State::getXPath()
{
	return xPath;
}

double State::getYPath()
{
	return yPath;
}

double State::getPsiPath()
{
	return psiPath;
}

double State::getVCmd()
{
	return vCmd;
}

double State::getSegDistDone()
{
	return segDistDone;
}

eecs_376_alpha::PathSegment State::getSegment()
{
	return currSeg;
}
