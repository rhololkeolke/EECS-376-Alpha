#!/usr/bin/env python

import roslib; roslib.load_manifest('look_ahead_alpha')

from nav_msgs.msg._GridCells import GridCells as GridCellsMsg
from geometry_msgs.msg._Point import Point as PointMsg
from msg_alpha.msg._PointList import PointList as PointListMsg

from math import ceil

RATE = 20

corner1 = None
corner2 = None
numCells = None

def closedListCallback(data):
    pass

def populateGrid(closedList, path, grid):
    '''
    Given a list of closed points and a path and a grid
    This function will fill in the new grid with the path and check if a new path
    needs to be calculated. If a new path needs to be calculated then the populated grid
    will be used in the calculations. Otherwise the populated grid will be discarded
    '''
    
    xLength = int(ceil(abs(corner1.x-corner2.x))/numCells)
    yLength = int(ceil(abs(corner1.y-corner2.y))/numCells)

    # fill in the squares in the grid that are included in the current path
    if path is not None:
        pass

    recalculate = False

    for point in closedList:
        # add the point to the grid
        # if the cell is occupied by a path point
        # then set the recalculate flag
        pass 

    return (recalculate,grid)

def createGrid():
    '''
    This method uses the specified corners and the numCells to
    create a 2d array that will store the values used in the astar
    search
    '''
    # empty list
    mapArray = list()

    for i in range(numCells):
        mapArray.append(list())
        for j in range(numCells):
            mapArray[i].append(0) # 0 is blank, 1 is path, -1 is obstacle

    return mapArray
            

def main():
    global corner1, corner2, numCells

    rospy.init_node('astar_alpha_main')

    # set the parameters specified within the launch files

    # corner 1 and corner 2 specify the area in the map
    # that astar will consider
    if rospy.has_param('corner1x'):
        corner1x = rospy.get_param('corner1x')
    else:
        corner1x = 0.0 #CHANGE ME

    if rospy.has_param('corner1y'):
        corner1y = rospy.get_param('corner1y')
    else:
        corner1y = 0.0 #CHANGE ME

    if rospy.has_param('corner2x'):
        corner2x = rospy.get_param('corner2x')
    else:
        corner2x = 10.0 #CHANGE ME

    if rospy.has_param('corner2y'):
        corner2y = rospy.get_param('corner2y')
    else:
        corner2y = 10.0 #CHANGE ME

    corner1 = PointMsg()
    corner1.x = corner1x
    corner1.y = corner1y

    corner2 = PointMsg()
    corner2.x = corner2x
    corner2.y = corner2y

    # numCells specifies the number of cells to divide the
    # specified astar search space into
    if rospy.has_param('numCells'):
        numCells = rospy.get_param('numCells')
    else:
        numCells = rospy.get_param('numCells')

    # topic that the node looks for the closed points on
    if rospy.has_param('inflatedTopic'):
        inflatedTopic = rospy.get_param('inflatedTopic')
    else:
        inflatedTopic = 'inflated_obstacles'

    rospy.Subscriber(inflatedTopic,GridCellsMsg,closedListCallback)

    rospy.spin()

if __name__ == '__main__':
    main()
