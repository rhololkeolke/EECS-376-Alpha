#!/usr/bin/env python

import roslib; roslib.load_manifest('pathplanner_alpha')
import rospy
import math as m

from msg_alpha.msg._PathSegment import PathSegment as PathSegmentMsg
from msg_alpha.msg._Obstacles import Obstacles as ObstaclesMsg
from msg_alpha.msg._SegStatus import SegStatus as SegStatusMsg
from msg_alpha.msg._PathList import PathList as PathListMsg
from msg_alpha.msg._PointList import PointList as PointListMsg
from std_msgs.msg._Bool import Bool as BoolMsg
from geometry_msgs.msg._Quaternion import Quaternion as QuaternionMsg

from tf.transformations import quaternion_from_euler,euler_from_quaternion

from math import cos,sin,pi,sqrt
from collections import deque

obs = ObstaclesMsg()

RATE = 20.0

lastSegComplete = 0
segAbort = False

segNumber = 0

pathList = PathListMsg()

desPoints = []

def segStatusCallback(data):
    global lastSegComplete
    global segAbort
    global segNumber

    if(segAbort is not True):
        segAbort = data.abort 
    
    lastSegComplete = data.lastSegComplete

    # see if there is anything to delete
    numToDelete = 0
    for i,seg in enumerate(pathlist):
        if(seg.seg_number <= lastSegComplete):
            numToDelete += 1
        else:
            break

    # delete the number of items from the front of the list
    # this is assuming the segment numbers are in order
    if(numToDelete > 0):
        del pathList.segments[0:numToDelete-1]

def pointListCallback(data):
    global desPoints

    if(data.new):
        pathList.segments = []
        desPoints = data.desPoints


def addSegToList(pathSeg):
    global segNumber

    segNumber += 1

    pathSeg.seg_number = segNumber
    pathSeg.max_speeds = .25
    pathSeg.min_speeds = 0
    pathSeg.accel_limit = .125
    pathSeg.decel_limit = -.125
    pathSeg.curvature = 0
    
    pathList.segments.append(pathSeg)

    naptime.sleep()


def main():
    global segNumber
    global segAbort
    global pathList

    rospy.init_node('path_planner_alpha_main')
    pathSegPub = rospy.Publisher('path', PathListMsg)
    rospy.Subscriber('seg_status', SegStatusMsg, segStatusCallback)
    rospy.Subscriber('point_list', PointListMsg, pointListCallback)

    naptime = rospy.Rate(RATE)

    while not rospy.is_shutdown():

        # clear everything when something gets in the way
        # of the planned path
        if(segAbort):
            segNumber = 0
            pathList.segments = []

        if(len(desPoints) >= segNumber+1):
            pathSeg = PathSegmentMsg()
            
            pathSeg.init_tan_angle = m.atan2((desPoints[segNumber+1].y-desPoints[segNumber].y),(desPoints[segNumber+1].x-desPoints[segNumber].x))

            #if(theta > m.pi/6):
                #SPIN TO NEW ANGLES
            #   addSegToList()

            #Calculate the length of the given segment
            #Add angle
            xDist = m.pow((desPoints[segNumber].x - desPoints[segNumber+1].x),2)
            yDist = m.pow((desPoints[segNumber].y - desPoints[segNumber+1].y),2)
            pathSeg.segLength = m.sqrt(xDist + yDist)

            addSegToList(pathSeg)


    pathSegPub.publish(pathList)

    naptime.sleep()


if __name__ == "__main__":
    main()