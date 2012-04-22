#!/usr/bin/env python                                                                                                                                                 

#ros imports
import roslib; roslib.load_manifest('astar_alpha')
import rospy
#ros node specifics


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
                    

# A* Search algorithim which computes the optimal path  and returns it as a stack
class Astar(object):
    def __init__(self):
        self.openList = Queue.priorityQueue(0) #0 queue size means infinte
        self.closedList = []
        self.closedList.append(self.closedList)
        self.nodes = []
        self.gridHeight = 84 + 6
        self.gridWidth = 60 + 18

        
        #initialize a grid of corordinates 
        def grid(self):
            closedList = ((0, 5), (1, 0), (1, 1), (1, 5), (2, 3)
           (3, 1), (3, 2), (3, 5), (4, 1), (4, 4), (5, 1))  #Mock closed list

            
            #Create a 2d grid array of nodes setting open nodes and closed nodes
            for x in range(self.gridWidth):
                for y in range(self.gridHeight):
                    if(x,y) in walls:
                        free = False
                        self.nodes.append(Node(x,y,free))
                    else:                                                           
                        free = True
                        self.nodes.append(Node(x,y,free))
            #the start grid is the first node and the last is the end
            self.start = (0,0) #(9,15)?
            self.end = (78,90) #(1,25)?
            
        #a function that computes the optimal path and returns the directions of as a stack
        def search(self):

                
                
            

                                                       



def main():
    rospy.init_node('n')  #initialize node with the name n                                                                                          
#    rospy.Subscriber("base_scan",LaserScan,laserCallback) #node subscribes to base_scan topic which is of type LaserScan which invokes the laserCallback with the msg as first arg                                                                                                                                          
 #   rospy.Subscriber("path_seg", PathSegmentMsg, pathSegCallback) #subscribe to path_seg topic of type PathSegmentMsg using PathSegCallback                        
    astart
    rospy.spin() 

