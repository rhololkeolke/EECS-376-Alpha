#!/usr/bin/env python                                                                                                                                                 

#ros imports
import roslib; roslib.load_manifest('astar_alpha')
import rospy
#ros node specifics
from sensor_msgs.msg import LaserScan
from msg_alpha.msg._PathSegment import PathSegment
from msg_alpha.msg._Obstacles import Obstacles
#mathematics
import math
import numpy
#data structures
import Queue



#A class that stores the x,y position of a node as well as its cost and heuristic and path cost
class Node():
    def __init__(self,x,y,free):
        self.x  = x
        self.y = y
        self.g = 0
        self.h = 0
        self.parent = None

        def getX(self):
            return self.x
        
        def getY(self):
            return self.y

        def getG(self):
            return self.g

        def getH(self):
            return self.h

        def getParent(self):
            return parent
        
        def getNode(self):
            return self.Node()
            

# A* Search algorithim which computes the optimal path  and returns it as a stack
class Astar(object):
    def __init__(self):
        self.openList = Queue.priorityQueue(0) #0 queue size means infinte
        self.closedList = []
        self.closedList.append(self.closedList)
        self.nodes[]
        self.gridHeight = 84 + 6
        self.gridWidth = 60 + 18

        
        def init_grid(self):
            closedList = ((0, 5), (1, 0), (1, 1), (1, 5), (2, 3)
           (3, 1), (3, 2), (3, 5), (4, 1), (4, 4), (5, 1))

            for x in range(self.gridWidth):
                for y in range(self.gridHeight):
                    if(x,y) in walls:
                        free = False
                    else:
                        free = True
                        self.cells.append(Node(x,y,free))
            self.start = Node(9,15,free)
            self.end = Node(1,25,free)
                                                       
        


def main():
    rospy.init_node('n')  #initialize node with the name n                                                                                          
    rospy.Subscriber("base_scan",LaserScan,laserCallback) #node subscribes to base_scan topic which is of type LaserScan which invokes the laserCallback with the msg as first arg                                                                                                                                          
    rospy.Subscriber("path_seg", PathSegmentMsg, pathSegCallback) #subscribe to path_seg topic of type PathSegmentMsg using PathSegCallback                        
    rospy.spin() 

