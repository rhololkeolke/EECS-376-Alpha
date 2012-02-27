/*
 * state.cpp
 *
 *  Created on: Feb 27, 2012
 *      Author: Devin
 */

#include "state.h"
#include <math.h>
#include <iostream>

using namespace std;

const double PI = 3.14159;

State::State()
{
	xPath = 0.0;
	yPath = 0.0;
	psiPath = 0.0;
	segDistDone = 0.0;
	currSeg = NULL;

	//sub = n.subscribe("base_pose_ground_truth",1,poseCallback);
}

State::State(eecs_376_alpha::PathSegment segment)
{
	xPath = segment.ref_point.x;
	yPath = segment.ref_point.y;
	psiPath = tf::getYaw(segment.init_tan_angle);
	currSeg = &segment; // may make this pass by reference so that velocity profiler can update the ending positions and velocities via callbacks

	//sub = n.subscribe("base_pose_ground_truth",1,poseCallback);
}

void State::updateState(double vCmd, double dt)
{
	double dL = vCmd*dt;
	double expSegDistDone = segDistDone + dL; // compute how much more the robot should complete based on previous segment distance done

	if(currSeg == NULL) // if no path segment currently defined
	{
		return; // do nothing
	}
	if(currSeg->seg_type == 1) // straight line segment
	{
		// compute the theoretical line position
		double psiPath = tf::getYaw(currSeg->init_tan_angle);
		double rhoDes = 0; // this is zero because lines have no curvature
		// line starts at xPath and yPath and is an an angle psiDes
		xPath = currSeg->ref_point.x+expSegDistDone*cos(psiPath);
		yPath = currSeg->ref_point.y+expSegDistDone*sin(psiPath);

		// get the actual line position
		//         May not actually need to do this here
		// project actual onto desired
		//         May not actually need to do this here
		// calculate the segDistDone
		segDistDone = expSegDistDone;
		if(segDistDone >= currSeg->seg_length)
		{
			currSeg = NULL;
		}
	}
	else if(currSeg->seg_type == 2) // arc segment
	{
		cout << "running seg_type 2" << endl;
		// calculate desired path
		double rhoDes = currSeg->curvature;
		double r = 1/fabs(rhoDes); // turn radius of inverse curvature
		double arcAngleStart;

		if(rhoDes >= 0)
			arcAngleStart = tf::getYaw(currSeg->init_tan_angle)-PI/2;
		else
			arcAngleStart = tf::getYaw(currSeg->init_tan_angle)+PI/2;

		double dAng = expSegDistDone*rhoDes;
		double arcAng = arcAngleStart+dAng;
		xPath = currSeg->ref_point.x + r*cos(arcAng);
		cout << "y: " << currSeg->ref_point.y << endl;
		cout << "r: " << r << endl;
		cout << "arcAng: " << arcAng << endl;
		yPath = currSeg->ref_point.y + r*sin(arcAng);
		psiPath = tf::getYaw(currSeg->init_tan_angle)+dAng;

		// calculate actual path
		//         May not actually need to do this here
		// compute projection of actual onto desired
		//         May not actually need to do this here
		// update segDistDone
		segDistDone = expSegDistDone;
		if(segDistDone >= currSeg->seg_length)
			currSeg = NULL;
	}
	else if(currSeg->seg_type == 3)
	{
		// calculate desired path
		psiPath = tf::getYaw(currSeg->init_tan_angle) + currSeg->curvature*expSegDistDone;

		// calculate actual path

		// compute projection of actual onto desired

		// update segDistDone
		segDistDone = expSegDistDone;
		if(segDistDone >= currSeg->seg_length)
			currSeg = NULL;
	}
}

// make segment the new current segment
void State::newSegment(eecs_376_alpha::PathSegment segment)
{
	// may also want to update x,y,and psi values
	segDistDone = 0.0;
	currSeg = &segment;
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

eecs_376_alpha::PathSegment* State::getSegment()
{
	return currSeg;
}
/*
void State::positionCallback(const nav_msgs::Odometry::ConstPtr& pose)
{
	geometry_msgs::Point position = pose->position;
	geometry_msgs::Quaternion orientation = pose->orientation;

	xAct = position.x;
	yAct = position.y;

	psiAct = tf::getYaw(orientation);
}
*/
