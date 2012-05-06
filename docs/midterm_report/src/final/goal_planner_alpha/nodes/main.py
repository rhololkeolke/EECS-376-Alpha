#!/usr/bin/env python  
import roslib ; roslib.load_mainfest('goal_planner_alpha')
import rospy
import tf

from geometry_msgs.msg._PoseStamped import PoseStamped as PoseStampedMsg
from geometry_msgs.msg._Point import Point as PointMsg
from msg_alpha.msg.Goal import Goal
from msg_alpha.msg._PointList import PointList as PointListMsg

RATE = 20.0
goalPoint = PointMsg()
goalList = []

from tf.transformations import transformPoint

def poseCallback(poseData):
    global pose
    pose = poseData


def findMagnet():

	TransformListener.transformDATATYPE (map_frame, PointMsg) 

def findPrize():
	

def determineGoal():
	global goalPoint

	if( abs(pose.pose.x - goalList[i].x) AND abs(pose.pose.y - goalList[i].y) < .1):
		i++
		if(goalList[i] > len(goalList)+1):
			goalPoint.none = True;
		goalPoint.point = goalList[i]


def main():
	global pose
	global goalList

	rospy.init_node('goal_planner_alpha')

	rospy.Subscriber('map_pos', PoseStampedMsg, poseCallback)
	goalPointPub = rospy.Publisher('goal_point', Goal)
    naptime = rospy.Rate(RATE)

	endOfObstacles = PointMsg()
	endOfObstacles.x = -3.48
	endOfObstacles.y = 20.69
	goalList.append(endOfObstacles)

	needsToTurn = PointMsg()
	needsToTurn.x = -2.37
	needsToTurn.y = 21.34
	goalList.append(needsToTurn)

	finalGoalPoint = PointMsg()
	finalGoalPoint.x = 5.47
	finalGoalPoint.y = 12.04
	goalList.append(finalGoalPoint)


	while not rospy.is_shutdown():

		goal = determineGoal()
		goalPointPub.publish(goalPoint)


		naptime.sleep()


if __name__ == "__main__":
	main()