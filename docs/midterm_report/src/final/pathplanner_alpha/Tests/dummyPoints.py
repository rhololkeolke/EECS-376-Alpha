#!/usr/bin/env python

import roslib; roslib.load_manifest('pathplanner_alpha')
import rospy
import csv

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

	pathList.points.append(point)

def main(fullList):
	global pose
	global pathList

	rospy.init_node('pathplanner_alpha_dummyNode')
	rospy.Subscriber('map_pos', PoseStampedMsg, poseCallback)
	dummyPointPub = rospy.Publisher('point_list', PointListMsg)


	pathList.new = True

	naptime = rospy.Rate(RATE)

	#point = PointMsg()
	#point.x = pose.pose.position.x
	#point.y = pose.pose.position.y
	#appendToList(point)

	with open(fullList, 'rb') as csvFile:
		dialect = csv.Sniffer().sniff(csvFile.read(1024)) # auto detect delimiters
		csvFile.seek(0)
		reader = csv.reader(csvFile, dialect) # open up a csv reader object with the csv file

		for i,row in enumerate(reader):

			point1 = PointMsg()
			try:
				point1.x = float(row[0])
			except ValueError:
				print "x ValueError"

			try:
				point1.y = float(row[1])
			except ValueError:
				print "y ValueError"
			
			appendToList(point1)
	

	while not rospy.is_shutdown(): 
            dummyPointPub.publish(pathList)
            pathList.new = False

            naptime.sleep()


if __name__ == "__main__":
    import sys
    import os
    
    if(len(sys.argv) > 1):
        fileName = sys.argv[1]
        fullPath = os.path.join('.',fileName)
        
        main(fullPath)
