/*
 * stateTest.cpp
 *
 *  Created on: Feb 27, 2012
 *      Author: Devin
 */

#include "../src/state.h"
#include <gtest/gtest.h>

TEST(TestSuite, testCase1)
{
	State state = State();

	state.updateState(4.0,0.0,.1);

	ASSERT_DOUBLE_EQ(0.2, state.getX());
	ASSERT_DOUBLE_EQ(0.0, state.getY());
}

TEST(TestSuite, testCase2)
{
	State state = State();

	state.updateState(4.0,4.0,.1);
	state.stop();
	ASSERT_DOUBLE_EQ(10, state.getVCmd());
	ASSERT_DOUBLE_EQ(10, state.getOCmd());
}

int main(int argc, char **argv)
{
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
