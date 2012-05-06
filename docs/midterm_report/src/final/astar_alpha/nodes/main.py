#!/usr/bin/env python

import roslib; roslib.load_manifest('astar_alpha')

import rospy

from nav_msgs.msg._GridCells import GridCells as GridCellsMsg
from geometry_msgs.msg._Point import Point as PointMsg
from geometry_msgs.msg._PoseStamped import PoseStamped as PoseStampedMsg
from msg_alpha.msg._PointList import PointList as PointListMsg
from msg_alpha.msg._Goal import Goal as GoalMsg

import re

from astar import Astar

from math import ceil, floor, sqrt

RATE = 20

position = None

corner1 = None
corner2 = None
numCells = None

searcher = None

newPath = True

wallPoints = []

# regex
localPattern = re.compile('costmap_local')
globalPattern = re.compile('costmap_global')

def goalCallback(data):
    global searcher, newPath
    
    # if there is no goal or the goal has changed and
    # the stay put flag is not set

    new = False

    if((searcher.goal is None or (searcher.goal[0] != data.goal.x or searcher.goal[1] != data.goal.y)) and not data.none):
        if(searcher is None or position is None):
            return
        searcher.start = (position.x,position.y)
        new = searcher.updateGoal((data.goal.x,data.goal.y))
    elif(data.none):
        # otherwise stay put so set goal to none
        searcher.goal = None
        if(len(searcher.path) > 0):
            # only need to run computePath if
            # there is an existing path
            searcher.computePath()
            new = True

    print "updated goal recomputed a path: %s" % new

    newPath = newPath or new

def inflatedObstaclesCallback(data):
    global searcher, newPath, wallPoints
    
    if re.search(globalPattern,str(data._connection_header)):
        print "Walls!"
        wallPoints = list()
        for point in data.cells:
            wallPoints.append((point.x,point.y))
    elif re.search(localPattern,str(data._connection_header)):
        print "Sensors"
        if(searcher is None or position is None):
            return

        # converting to the tuple format is really inefficient
        # should change the astar method to expect to be able to
        # access the coorindates with .x and .y
        closedPoints = list()
        for point in data.cells:
            searcher.start = (position.x,position.y)
            closedPoints.append((point.x,point.y))
            
            closedPoints = closedPoints + wallPoints
            
            new = searcher.updateClosedList(closedPoints)

            print "inflated obstacles recomputed a path: %s" % new

            newPath = newPath or new

def poseCallback(pose):
    '''
    Updates the robot's best estimate on position and orientation
    '''
    global position
    position = pose.pose.position

def main():
    global corner1, corner2, numCells
    global searcher, newPath

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
        inflatedTopic = '/costmap_local_alpha/costmap_local/inflated_obstacles'

    # topic that the node looks for goal messages on
    if rospy.has_param('goalTopic'):
        goalTopic = rospy.get_param('goalTopic')
    else:
        goalTopic = 'goal_point'

    # initialize an instance of the Astar class
    searcher = Astar(corner1,corner2,numCells)
    naptime = rospy.Rate(RATE)
    
    print "corner1: "
    print corner1
    print ""
    print "corner2: "
    print corner2
    print ""
    print "numCells: %i" % numCells
    print ""
    print "goal topics: %s" % goalTopic
    print ""
    print "inflatedTopic: %s" % inflatedTopic

    rospy.Subscriber(goalTopic,GoalMsg,goalCallback)
    rospy.Subscriber(inflatedTopic,GridCellsMsg,inflatedObstaclesCallback)
    rospy.Subscriber('map_pos', PoseStampedMsg, poseCallback)

    pathPointPub = rospy.Publisher('point_list', PointListMsg)

    first_run = True
    pointList = PointListMsg()
    while not rospy.is_shutdown():
        pointList.new = newPath
        
        print "searcher.start"
        print searcher.start
        print ""
        print "searcher.goal"
        print searcher.goal
        print ""
        print "newPath"
        print newPath
        print ""
        print "searcher.path"
        print searcher.path
        print ""

        pointList.points = []
        for point in searcher.path:
            pathPoint = PointMsg()
            pathPoint.x = point[0]
            pathPoint.y = point[1]
            pointList.points.append(pathPoint)

        pathPointPub.publish(pointList)

        if newPath:
            newPath = False            

        naptime.sleep()

if __name__ == '__main__':
    main()
