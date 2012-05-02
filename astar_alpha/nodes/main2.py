#!/usr/bin/env python

import roslib; roslib.load_manifest('look_ahead_alpha')

from nav_msgs.msg._GridCells import GridCells as GridCellsMsg
from geometry_msgs.msg._Point import Point as PointMsg
from msg_alpha.msg._PointList import PointList as PointListMsg
from msg_alpha.msg._Goal import Goal as GoalMsg

from astar import Astar

from math import ceil, floor, sqrt

RATE = 20

position = None

corner1 = None
corner2 = None
numCells = None

searcher = None

newPath = True

def goalCallback(data):
    global searcher, newPath
    if(data.new):
        if(searcher is None):
            return
        searcher.start = (position.x,position.y)
        newPath = searcher.updateGoal((data.goal.x,data.goal.y))

def inflatedObstaclesCallback(data):
    global searcher, newPath
    
    if(searcher is None):
        return

    # converting to the tuple format is really inefficient
    # should change the astar method to expect to be able to
    # access the coorindates with .x and .y
    closedPoint = list()
    for point in data.cells:
        searcher.start = (position.x,position.y)
        closedPoints.append((point.x,point.y))

    newPath = searcher.updateClosedList(closedPoints)

def poseCallback(pose):
    '''
    Updates the robot's best estimate on position and orientation
    '''
    global position
    position = pose.pose.position

def main():
    global corner1, corner2, numCells
    global searcher

    rospy.init_node('astar_alpha_main')

    # set the parameters specified within the launch files

    # corner 1 and corner 2 specify the area in the map
    # that astar will consider

    # Corner1
    if rospy.has_param('corner1x'):
        corner1x = rospy.get_param('corner1x')
    else:
        corner1x = -6.25

    if rospy.has_param('corner1y'):
        corner1y = rospy.get_param('corner1y')
    else:
        corner1y = 8.2

    # Corner2
    if rospy.has_param('corner2x'):
        corner2x = rospy.get_param('corner2x')
    else:
        corner2x = 15.75

    if rospy.has_param('corner2y'):
        corner2y = rospy.get_param('corner2y')
    else:
        corner2y = 28.2

    corner1 = (corner1x,corner1y)

    corner2 = (corner2x,corner2y)


    # numCells specifies the number of cells to divide the
    # specified astar search space into
    if rospy.has_param('numCells'):
        numCells = rospy.get_param('numCells')
    else:
        numCells = 100

    # topic that the node looks for the closed points on
    if rospy.has_param('inflatedTopic'):
        inflatedTopic = rospy.get_param('inflatedTopic')
    else:
        inflatedTopic = '/costmap_alpha/costmap/inflated_obstacles'

    # topic that the node looks for goal messages on
    if rospy.has_param('goalTopic'):
        goalTopic = rospy.get_param('goalTopic')
    else:
        goalTopic = 'goal_point'

    # initialize an instance of the Astar class
    searcher = Astar(corner1,corner2,numCells)
    naptime = rospy.Rate(RATE)
    
    rospy.Subscriber(goalTopic,GoalMsg,goalCallback)
    rospy.Subscriber(inflatedTopic,GridCellsMsg,inflatedObstaclesCallback)

    pathPointPub = rospy.Publisher('point_list', PointListMsg)

    pointList = PointListMsg()
    while not rospy.is_shutdown():
        pointList.new = newPath
        pointList.points = searcher.path
        if newPath:
            newPath = False

        pathPointPub.publish(pointList)

        naptime.sleep()

if __name__ == '__main__':
    main()
