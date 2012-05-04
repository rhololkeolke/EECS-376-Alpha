#!/usr/bin/env python

import roslib; roslib.load_manifest('brushfire_alpha')

import rospy

from nav_msgs.msg._GridCells import GridCells as GridCellsMsg
from geometry_msgs.msg._Point import Point as PointMsg
from geometry_msgs.msg._PoseStamped import PoseStamped as PoseStampedMsg
from msg_alpha.msg._PointList import PointList as PointListMsg
from msg_alpha.msg._Goal import Goal as GoalMsg

from brushfire import BrushFire

from math import ceil, floor, sqrt
from threading import Timer

RATE = 20

position = None

corner1 = None
corner2 = None
numCells = None
pointList = PointListMsg()

brush = None

def goalCallback(data):
    global brush
    
    new = False
    if((brush.goal is None or (brush.goal[0] != data.goal.x or brush.goal[1] != data.goal.y)) and not data.none):
        if(brush is None):
            return
        brush.goal = (data.goal.x,data.goal.y)
        print "Updated goal to (%f, %f)" % (data.goal.x,data.goal.y)

def obstaclesCallback(data):
    global brush

    if(brush is None or position is None):
        return
    
    obstacles = list()
    for point in data.cells:
        obstacles.append((point.x,point.y))
        brush.updateGlobalGrid(obstacles)

def poseCallback(pose):
    global position
    position = pose.pose.position

def resetPath():
    pointlist.new = True
    t = Timer(2.0, resetPath)
    t.start()

def main():
    global corner1, corner2, numCells
    global brush
    global position
    global pointList

    rospy.init_node('brushfire_alpha_main')

    corner1 = (-6.25,8.2)
    corner2 = (15.75,28.2)
    numCells = 100

    brush = BrushFire(corner1,corner2,numCells,size=15)
    naptime = rospy.Rate(RATE)

    print "corner1: "
    print corner1
    print ""
    print "corner2: "
    print corner2
    print ""
    print "numCels: "
    print numCells
    print ""

    rospy.Subscriber('goal_point',GoalMsg,goalCallback)
    rospy.Subscriber('/costmap_alpha/costmap/obstacles', GridCellsMsg,obstaclesCallback)
    rospy.Subscriber('map_pos',PoseStampedMsg, poseCallback)

    pathPointPub = rospy.Publisher('point_list',PointListMsg)

    pointList = PointListMsg()

    t = Timer(2.0, resetPath)
    t.start()
    while not rospy.is_shutdown():
        if(position is None or brush is None or brush.goal is None):
            naptime.sleep()
            continue


        brush.extractLocal(position.x,position.y)
        brush.brushfire()
        brush.computePath()
        pointList.points = []
        for point in brush.pathList:
            pathPoint = PointMsg()
            pathPoint.x = point[0]
            pathPoint.y = point[1]
            pointList.points.append(pathPoint)

        pathPointPub.publish(pointList)
        pointList.new = False

        naptime.sleep()

if __name__ == '__main__':
    main()
