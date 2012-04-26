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
	global pathList

	pathList.cells.append(point)

def main():
	global pose
	global pathList

	rospy.init_node('pathplanner_alpha_dummyNode')
	rospy.Subscriber('map_pos', PoseStampedMsg, poseCallback)
	dummyPointPub = rospy.Publisher('point_list', PointListMsg)

	pathList.new = True

	naptime = rospy.Rate(RATE)

	point = PointMsg()
	point.x = pose.pose.position.x
	point.y = pose.pose.position.y
	appendToList(point)

	point1 = PointMsg()
	point1.x = 3.70
	point1.y = 13.00
	appendToList(point1)
	
	point2 = PointMsg()
	point2.x = 3.70
	point2.y = 13.75
	appendToList(point2)

	point3 = PointMsg()
	point3.x = 3.00
	point3.y = 13.95
	appendToList(point3)
	
	point4 = PointMsg()
	point4.x = 2.60
	point4.y = 14.40
	appendToList(point4)
	

	point5 = PointMsg()
	point5.x = 2.20
	point5.y = 15.05
	appendToList(point5)

	while not rospy.is_shutdown(): 
	


		dummyPointPub.publish(pathList)
		pathList.new = False

		naptime.sleep()


if __name__ == "__main__":
    main()	