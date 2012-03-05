/*
 * stateTest.cpp
 *
 *  Created on: Feb 27, 2012
 *      Author: Devin
 */

#include "../src/state.h"
#include <eecs_376_alpha/PathSegment.h>
#include <geometry_msgs/Twist.h>
#include <gtest/gtest.h>
#include <math.h>

#define PI 3.14159

TEST(StateTestSuite, perfectStraight)
{
	eecs_376_alpha::PathSegment seg;

	seg.curvature = 0.0;
	seg.seg_length = 1.0;
	seg.ref_point.x = 0.0;
	seg.ref_point.y = 0.0;
	seg.seg_number = 123;
	seg.seg_type = 1;
	seg.init_tan_angle = tf::createQuaternionMsgFromYaw(PI/4);
	seg.max_speeds.linear.x = 1.0;
	seg.max_speeds.angular.z = 1.0;
	seg.accel_limit=.5;
	seg.decel_limit = .5;

	geometry_msgs::Point initPos;
	initPos.x = 0.0;
	initPos.y = 0.0;
	initPos.z = 0.0;

	geometry_msgs::Twist vel;
	vel.linear.x = 1.0;
	vel.angular.z = 0;

	State state = State(.1,initPos,0.0,&seg);

	ASSERT_NEAR(0.0,state.getXPath(),.001);
	ASSERT_NEAR(0.0,state.getYPath(),.001);
	ASSERT_NEAR(0.0,state.getSegDistDone(),.001);
	ASSERT_NEAR(seg.curvature,state.getSegment()->curvature,.001);
	ASSERT_NEAR(seg.seg_length,state.getSegment()->seg_length,.001);
	ASSERT_NEAR(seg.ref_point.x,state.getSegment()->ref_point.x,.001);
	ASSERT_NEAR(seg.ref_point.y,state.getSegment()->ref_point.y,.001);
	ASSERT_EQ(seg.seg_number,state.getSegment()->seg_number);
	ASSERT_EQ(seg.seg_type,state.getSegment()->seg_type);
	ASSERT_NEAR(tf::getYaw(seg.init_tan_angle),tf::getYaw(state.getSegment()->init_tan_angle),.001);
	ASSERT_NEAR(seg.max_speeds.linear.x,state.getSegment()->max_speeds.linear.x,.001);
	ASSERT_NEAR(seg.max_speeds.angular.z,state.getSegment()->max_speeds.angular.z,.001);
	ASSERT_NEAR(seg.accel_limit,state.getSegment()->accel_limit,.001);
	ASSERT_NEAR(seg.decel_limit,state.getSegment()->decel_limit,.001);

	for(int i=0; i<10; i++)
	{
		state.updateState(vel);

		ASSERT_NEAR((double)(i+1) * .1*cos(PI/4),state.getXPath(),.0001);
		ASSERT_NEAR(state.getXPath(), state.getYPath(),.0001);
		ASSERT_NEAR(PI/4,state.getPsiPath(),.0001);
		ASSERT_NEAR((double)(i+1)*0.1,state.getSegDistDone(),.1);
	}
	//EXPECT_EQ(NULL,state.getSegment());
	state.updateState(1.0,.1);
	ASSERT_EQ(NULL,state.getSegment());
}

TEST(StateTestSuite, perfectArc)
{
	eecs_376_alpha::PathSegment seg;

	seg.curvature = 0.5;
	seg.seg_length = 1.0;
	seg.ref_point.x = 0.0;
	seg.ref_point.y = 0.0;
	seg.seg_number = 123;
	seg.seg_type = 2;
	seg.init_tan_angle = tf::createQuaternionMsgFromYaw(0.0);
	seg.max_speeds.linear.x = 1.0;
	seg.max_speeds.angular.z = 1.0;
	seg.accel_limit=.5;
	seg.decel_limit = .5;

	geometry_msgs::Point initPos;
	initPos.x = 0.0;
	initPos.y = 0.0;
	initPos.z = 0.0;

	State state = State(.1,initPos,0.0,&seg);
	geometry_msgs::Twist vel;
	vel.linear.x = 1.0;
	vel.angular.z = 0;

	ASSERT_NEAR(0.0,state.getXPath(),.001);
	ASSERT_NEAR(0.0,state.getYPath(),.001);
	ASSERT_NEAR(0.0,state.getSegDistDone(),.001);
	ASSERT_NEAR(seg.curvature,state.getSegment()->curvature,.001);
	ASSERT_NEAR(seg.seg_length,state.getSegment()->seg_length,.001);
	ASSERT_NEAR(seg.ref_point.x,state.getSegment()->ref_point.x,.001);
	ASSERT_NEAR(seg.ref_point.y,state.getSegment()->ref_point.y,.001);
	ASSERT_EQ(seg.seg_number,state.getSegment()->seg_number);
	ASSERT_EQ(seg.seg_type,state.getSegment()->seg_type);
	ASSERT_NEAR(tf::getYaw(seg.init_tan_angle),tf::getYaw(state.getSegment()->init_tan_angle),.001);
	ASSERT_NEAR(seg.max_speeds.linear.x,state.getSegment()->max_speeds.linear.x,.001);
	ASSERT_NEAR(seg.max_speeds.angular.z,state.getSegment()->max_speeds.angular.z,.001);
	ASSERT_NEAR(seg.accel_limit,state.getSegment()->accel_limit,.001);
	ASSERT_NEAR(seg.decel_limit,state.getSegment()->decel_limit,.001);

	for(int i=0;i<10;i++)
	{
		state.updateState(vel);

		ASSERT_NEAR((1/seg.curvature)*cos(tf::getYaw(seg.init_tan_angle)-PI/2+(double)(i+1)*.1*seg.curvature),state.getXPath(),.0001);
		ASSERT_NEAR((1/seg.curvature)*sin(tf::getYaw(seg.init_tan_angle)-PI/2+(double)(i+1)*.1*seg.curvature), state.getYPath(),.0001);
		ASSERT_NEAR(tf::getYaw(seg.init_tan_angle)+(double)(i+1)*.1*seg.curvature,state.getPsiPath(),.0001);
		ASSERT_NEAR((double)(i+1)*0.1,state.getSegDistDone(),.1);
	}
	//EXPECT_EQ(NULL,state.getSegment());
	state.updateState(1.0,.1);
	ASSERT_EQ(NULL,state.getSegment());
}

TEST(StateTestSuite, perfectSpin)
{
	eecs_376_alpha::PathSegment seg;

	seg.curvature = 1.0;
	seg.seg_length = PI/4;
	seg.ref_point.x = 0.0;
	seg.ref_point.y = 0.0;
	seg.seg_number = 123;
	seg.seg_type = 3;
	seg.init_tan_angle = tf::createQuaternionMsgFromYaw(0.0);
	seg.max_speeds.linear.x = 1.0;
	seg.max_speeds.angular.z = 1.0;
	seg.accel_limit=.5;
	seg.decel_limit = .5;

	geometry_msgs::Point initPos;
	initPos.x = 0.0;
	initPos.y = 0.0;
	initPos.z = 0.0;

	geometry_msgs::Twist vel;
	vel.linear.x = 0.0;
	vel.angular.z = PI/4.0;

	State state = State(.1,initPos,0.0,&seg);

	ASSERT_NEAR(0.0,state.getXPath(),.0001);
	ASSERT_NEAR(0.0,state.getYPath(),.0001);
	ASSERT_NEAR(0.0,state.getSegDistDone(),.0001);
	ASSERT_NEAR(seg.curvature,state.getSegment()->curvature,.0001);
	ASSERT_NEAR(seg.seg_length,state.getSegment()->seg_length,.0001);
	ASSERT_NEAR(seg.ref_point.x,state.getSegment()->ref_point.x,.0001);
	ASSERT_NEAR(seg.ref_point.y,state.getSegment()->ref_point.y,.0001);
	ASSERT_EQ(seg.seg_number,state.getSegment()->seg_number);
	ASSERT_EQ(seg.seg_type,state.getSegment()->seg_type);
	ASSERT_NEAR(tf::getYaw(seg.init_tan_angle),tf::getYaw(state.getSegment()->init_tan_angle),.0001);
	ASSERT_NEAR(seg.max_speeds.linear.x,state.getSegment()->max_speeds.linear.x,.0001);
	ASSERT_NEAR(seg.max_speeds.angular.z,state.getSegment()->max_speeds.angular.z,.0001);
	ASSERT_NEAR(seg.accel_limit,state.getSegment()->accel_limit,.0001);
	ASSERT_NEAR(seg.decel_limit,state.getSegment()->decel_limit,.0001);

	for(int i=0; i<10; i++)
	{
		state.updateState(vel);

		ASSERT_NEAR(state.getXPath(),0.0,.0001);
		ASSERT_NEAR(state.getYPath(), 0.0,.0001);
		ASSERT_NEAR((double)(i+1)*.1*PI/4+tf::getYaw(seg.init_tan_angle),state.getPsiPath(),.1);
		ASSERT_NEAR((double)(i+1)*0.1*PI/4,state.getSegDistDone(),.1);
	}
	//EXPECT_EQ(NULL,state.getSegment());
	state.updateState(PI/4,.1);
	ASSERT_EQ(NULL,state.getSegment());
}

TEST(StateTestSuite, emptyConstructor)
{
	State state = State();

	ASSERT_EQ(NULL,state.getSegment());

	ASSERT_NEAR(0.0,state.getXPath(),.0001);
	ASSERT_NEAR(0.0,state.getYPath(),.0001);
	ASSERT_NEAR(0.0,state.getPsiPath(),.0001);
	ASSERT_NEAR(0.0,state.getSegDistDone(),.0001);
}

int main(int argc, char **argv)
{
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
