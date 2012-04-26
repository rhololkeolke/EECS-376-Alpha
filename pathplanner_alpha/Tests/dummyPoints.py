#!/usr/bin/env python

import roslib; roslib.load_manifest('pathplanner_alpha')
import rospy

from geometry_msgs.msg._PoseStamped import PoseStamped as PoseStampedMsg
from geometry_msgs.msg._Point import Point as PointMsg
from msg_alpha.msg._PointList import PointList as PointListMsg

RATE = 20.0

pathList = PointListMsg()

pose = PoseStampedMsg()

def poseCallback(poseData):
    global pose
    pose = poseData

def appendToList(point):

	pathList.cells.append(point)

def main():
	global pose

	rospy.init_node('pathplanner_alpha_dummyNode')
	rospy.Subscriber('map_pos', PoseStampedMsg, poseCallback)
	dummyPointPub = rospy.Publisher('point_list', PointListMsg)

	naptime = rospy.Rate(RATE)

	while not rospy.is_shutdown(): 
	
		point = PointMsg()
		point.x = pose.pose.position.x
		point.y = pose.pose.position.y
		appendToList(point)

		point = PointMsg()
		point.x = 3.70
		point.y = 13.00
		appendToList(point)

		point.x = 3.70
		point.y = 13.75
		appendToList(point)
		
		point.x = 3.00
		point.y = 13.95
		appendToList(point)
		
		point.x = 2.60
		point.y = 14.40
		appendToList(point)
		
		point.x = 2.20
		point.y = 15.05
		appendToList(point)

		dummyPointPub.publish(pathList)

		naptime.sleep()


if __name__ == "__main__":
    main()	