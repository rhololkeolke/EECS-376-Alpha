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

// this is kind of useless, except when currState is initialized
State::State()
{
	// set the dt to 0.0;
	dt = 0.0;

	// set ideal point on path to 0's
	pathPos.x = 0.0;
	pathPos.y = 0.0;
	pathPos.z = 0.0;
	pathPsi = 0.0;

	// set the actual point to 0's
	actPos.x = 0.0;
	actPos.y = 0.0;
	actPos.z = 0.0;
	actPsi = 0.0;

	// set the initial velocity commands to 0
	velocity.linear.x = 0.0;
	velocity.angular.z = 0.0;

	// set segDone to 0
	segDone = 0.0;

	// set the current segment to NULL
	currSeg = NULL;
}

State::State(double dtDes, geometry_msgs::Point initPos, double initPsi, eecs_376_alpha::PathSegment *segment)
{
	// set dt
	dt = dtDes;

	// initialize the path position and heading to the desired position and heading
	pathPos.x = segment->ref_point.x;
	pathPos.y = segment->ref_point.y;
	pathPos.z = segment->ref_point.z; // technically this isn't needed, but just to be safe its in here
	pathPsi = tf::getYaw(segment->init_tan_angle); // extract the initial heading in radians from the segment

	// initialize the actual position and heading
	actPos = initPos; // simply copy the datastructure
	actPsi = initPsi;

	// set the segDone to 0
	segDone = 0.0;

	// set the current segment to the one passed in
	currSeg = segment;
}

void State::updateState(geometry_msgs::Twist vel)
{
	velocity = vel; // vel.linear.x is the vCmd and vel.angular.z is the oCmd
	if(currSeg == NULL)
		return; // can't do any propagation without a path segment

	double dL; // will store the ideal amount moved along the path
	if(currSeg->seg_type == 1 || currSeg->seg_type == 2)
		dL = vel.linear.x*dt; // assume the oCmd is set to match desired headings/curvatures for straights/arcs, so that vCmd is velocity along the path segment
	else if(currSeg->seg_type == 3)
		dL = vel.angular.z*dt; // assuming that vCmd is 0 for spin segments
	else
	{
		cout << "Unknown path type" << endl;
		return; // can't do anything if the path type isn't known
	}

	segDone += dL; // compute how ideal segment distance done

	// already checked if currSeg is NULL so this is okay to not check again
	if(currSeg->seg_type == 1) // straight line segment
	{
		// compute the theoretical line position
		pathPsi = tf::getYaw(currSeg->init_tan_angle);
		double rhoDes = 0; // this is zero because lines have no curvature
		// line starts at xPath and yPath and is an an angle psiDes
		pathPos.x = currSeg->ref_point.x+SegDone*cos(psiPath);
		pathPos.y = currSeg->ref_point.y+SegDone*sin(psiPath);

		if(segDone >= currSeg->seg_length)
		{
			currSeg = NULL;
		}
	}
	else if(currSeg->seg_type == 2) // arc segment
	{
		// calculate desired path
		double rhoDes = currSeg->curvature;
		double r = 1/fabs(rhoDes); // turn radius of inverse curvature
		double arcAngleStart;

		if(rhoDes >= 0)
		{
			arcAngleStart = tf::getYaw(currSeg->init_tan_angle)-PI/2;
		}
		else
		{
			arcAngleStart = tf::getYaw(currSeg->init_tan_angle)+PI/2;
		}

		double dAng = SegDone*rhoDes;
		double arcAng = arcAngleStart+dAng;

		pathPos.x = currSeg->ref_point.x + r*cos(arcAng);
		pathPos.y = currSeg->ref_point.y + r*sin(arcAng);
		pathPsi = tf::getYaw(currSeg->init_tan_angle)+dAng;

		if(segDone >= currSeg->seg_length)
			currSeg = NULL;
	}
	else if(currSeg->seg_type == 3)
	{
		// calculate desired path
		pathPsi = tf::getYaw(currSeg->init_tan_angle) + currSeg->curvature*expSegDistDone;

		if(segDone >= currSeg->seg_length)
			currSeg = NULL;
	}
}

void State::updateState(geometry_msgs::Twist vel, geometry_msgs::Point pos, double psi)
{
	velocity = vel; // vel.linear.x is the vCmd and vel.angular.z is the oCmd
	actPos = pos;
	actPsi = psi;

	if(currSeg == NULL)
		return; // can't do any propagation without a path segment

	double dL; // will store the ideal amount moved along the path
	if(currSeg->seg_type == 1 || currSeg->seg_type == 2)
		dL = vel.linear.x*dt; // assume the oCmd is set to match desired headings/curvatures for straights/arcs, so that vCmd is velocity along the path segment
	else if(currSeg->seg_type == 3)
		dL = vel.angular.z*dt; // assuming that vCmd is 0 for spin segments
	else
	{
		cout << "Unknown path type" << endl;
		return; // can't do anything if the path type isn't known
	}

	double expSegDone = segDone + dL; // compute how ideal segment distance done

	geometry_msgs::Vector3 act_r;
	geometry_msgs::Vector3 des_r;
	geometry_msgs::Vector3 des_e;

	geometry_msgs::Point newPos;

	// already checked if currSeg is NULL so this is okay to use
	if(currSeg->seg_type == 1) // straight line segment
	{
		// compute the theoretical line position
		pathPsi = tf::getYaw(currSeg->init_tan_angle);
		double rhoDes = 0; // this is zero because lines have no curvature
		// line starts at xPath and yPath and is an an angle psiDes
		newPos.x = currSeg->ref_point.x+expSegDistDone*cos(psiPath);
		newPos.y = currSeg->ref_point.y+expSegDistDone*sin(psiPath);
		newPos.z = 0.0;

		// calculate unit vector in desired direction
		des_r = getVector(pathPos,newPos);
		des_e = unitVector(des_r);

		// calculate actual vector of motion
		act_r = getVector(pathPos,actPos);

		// project actual onto desired
		double act_dL = dotProduct(act_r, des_e);

		// calculate the segDistDone
		segDone += act_dL;
		if(segDone >= currSeg->seg_length)
		{
			currSeg = NULL;
		}
	}
	else if(currSeg->seg_type == 2) // arc segment
	{
		// calculate desired path
		double rhoDes = currSeg->curvature;
		double r = 1/fabs(rhoDes); // turn radius of inverse curvature
		double arcAngleStart;

		if(rhoDes >= 0)
		  {
			arcAngleStart = tf::getYaw(currSeg->init_tan_angle)-PI/2;
		  }
		else
		  {
		  arcAngleStart = tf::getYaw(currSeg->init_tan_angle)+PI/2;
		  }

		double dAng = expSegDistDone*rhoDes;
		double arcAng = arcAngleStart+dAng;

		newPos.x = currSeg->ref_point.x + r*cos(arcAng);
		newPos.y = currSeg->ref_point.y + r*sin(arcAng);
		double newPsi = tf::getYaw(currSeg->init_tan_angle)+dAng;

		// calculate unit vector in desired direction
		des_r = getVector(pathPos,newPos);
		des_e = unitVector(des_r);

		// calculate actual vector of motion
		act_r = getVector(pathPos,actPos);

		// project actual onto desired
		double act_dL = dotProduct(act_r, des_e);
		// update segDistDone
		segDistDone += act_dL;
		if(segDistDone >= currSeg->seg_length)
			currSeg = NULL;
	}
	else if(currSeg->seg_type == 3)
	{
		// segDistDone is simply the distance turned since the last move
		double act_dL = actPsi - pathPsi;
		pathPsi = actPsi;

		// update segDistDone
		segDistDone += act_dL;
		if(segDistDone >= currSeg->seg_length)
			currSeg = NULL;
	}
}

// make segment the new current segment
void State::newSegment(eecs_376_alpha::PathSegment *segment)
{
	// may also want to update x,y,and psi values
	segDistDone = 0.0;
	currSeg = segment; // this is assuming that the memory will be dealt with appropriately outside of this class, passing by reference or copying may be better
}

void State::stop()
{
	velocity.linear.x = 0.0;
	velocity.angular.z = 0.0;
}

geometry_msgs::Point getPathPoint()
{
	return pathPos;
}

double State::getXPath()
{
	return pathPos.x;
}

double State::getYPath()
{
	return pathPos.y;
}

double State::getPsiPath()
{
	return pathPsi;
}

geometry_msgs::Point State::getActPoint()
{
	return actPos;
}

double State::getXAct()
{
	return actPos.x;
}

double State::getYAct()
{
	return actPos.y;
}

double State::getPsiAct()
{
	return actPsi;
}

geometry_msgs::Twist State::getVelocity()
{
	return velocity;
}

double State::getVCmd()
{
	return velocity.linear.x;
}

double State::getOCmd()
{
	return velocity.angular.y;
}

double State::getSegDistDone()
{
	return segDone;
}

eecs_376_alpha::PathSegment* State::getSegment()
{
	return currSeg;
}

// Static Methods

static geometry_msgs::Vector3 State::getVector(geometry_msgs::Point p0, geometry_msgs::Point p1)
{
	geometry_msgs::Vector3 vec;
	vec.x = p1.x - p0.x;
	vec.y = p1.y - p0.y;
	vec.z = p1.z - p0.z;

	return vec;
}

static geometry_msgs::Vector3 State::unitVector(geometry_msgs::Vector3 r)
{
	geometry_msgs::Vector3 e;

	double length = sqrt(pow(r.x,2) + pow(r.y,2) + pow(r.z,2));

	e.x = r.x/length;
	e.y = r.y/length;
	e.z = r.z/length;

	return e;
}

static double dotProduct(geometry_msgs::Vector3 r0, geometry_msgs::Vector3 r1)
{
	double result = 0.0;
	result += r1.x*r0.x;
	result += r1.y*r0.y;
	result += r1.z*r0.z;

	return result;
}
