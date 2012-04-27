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

RATE = 20.0

obs = ObstaclesMsg()

RATE = 20.0

naptime = None

lastSegComplete = 0
segAbort = False
last_seg = 1

segNumber = 0

pathList = PathListMsg()

desPoints = []

def yawToQuat(angle):
    quatList = quaternion_from_euler(0.0,0.0,angle)
    quat = QuaternionMsg()
    quat.x = quatList[0]
    quat.y = quatList[1]
    quat.z = quatList[2]
    quat.w = quatList[3]
    return quat

def segStatusCallback(data):
    global lastSegComplete
    global segAbort
    global segNumber
    global pathList

    if(segAbort is not True):
        segAbort = data.abort 
    
    lastSegComplete = data.lastSegComplete

    # see if there is anything to delete
    numToDelete = 0
    for i,seg in enumerate(pathList.segments):
        if(seg.seg_number <= lastSegComplete):
            numToDelete += 1
        else:
            break

    # delete the number of items from the front of the list
    # this is assuming the segment numbers are in order
    if(numToDelete > 1):
        del pathList.segments[0:numToDelete-1]
    elif(numToDelete == 1):
        del pathList.segments[0]

def pointListCallback(data):
    global desPoints
    global pathList

    #print "In the point list Call back now"

    if(data.new or desPoints == []):
        pathList.segments = []
        desPoints = data.points


def addSegToList(pathSeg):
    global segNumber
    global pathList

    segNumber += 1

    pathSeg.seg_number = segNumber
    pathSeg.seg_type = 1
    pathSeg.max_speeds.linear.x = .25
    pathSeg.min_speeds.linear.x = 0
    pathSeg.accel_limit = .125
    pathSeg.decel_limit = -.125
    pathSeg.curvature = 0
    
    pathList.segments.append(pathSeg)

    #print pathList

    naptime.sleep()


def main():
    global segNumber
    global segAbort
    global pathList
    global naptime

    rospy.init_node('path_planner_alpha_main')
    pathSegPub = rospy.Publisher('path', PathListMsg)
    rospy.Subscriber('seg_status', SegStatusMsg, segStatusCallback)
    rospy.Subscriber('point_list', PointListMsg, pointListCallback)

    naptime = rospy.Rate(RATE)

    print "Entering main loop"

    while not rospy.is_shutdown():

        # clear everything when something gets in the way
        # of the planned path
        if(segAbort):
            segNumber = 0
            pathList.segments = []

        if(len(desPoints) > segNumber+1):
            pathSeg = PathSegmentMsg()
            
            
            yaw = m.atan2((desPoints[segNumber+1].y-desPoints[segNumber].y),(desPoints[segNumber+1].x-desPoints[segNumber].x))

            pathSeg.init_tan_angle = yawToQuat(yaw)
            pathSeg.ref_point = desPoints[segNumber]

            #if(theta > m.pi/6):
                #SPIN TO NEW ANGLES
            #   addSegToList()

            #Calculate the length of the given segment
            #Add angle
            xDist = m.pow((desPoints[segNumber].x - desPoints[segNumber+1].x),2)
            yDist = m.pow((desPoints[segNumber].y - desPoints[segNumber+1].y),2)
            pathSeg.seg_length = m.sqrt(xDist + yDist)

            addSegToList(pathSeg)


        pathSegPub.publish(pathList)

        naptime.sleep()


if __name__ == "__main__":
    main()
