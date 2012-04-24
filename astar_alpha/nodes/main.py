#!/usr/bin/env python                                                                                                                                                 

'''
This program uses the A* search algorithim to find the optimal path on a grid map given a starting point and an ending point.
Each node of the grid is specified by a Node class which stores the x,y position, and cost to move from one node to another node.

The Astar class handles the start position and goal for a search as well as computing the heuristic function for a given node.
The node is published as an array of points

@author EECS-376-Alpha-Eddie Massey III

'''

#ros imports
import roslib; roslib.load_manifest('astar_alpha')
import rospy
#ros package imports

#mathematics
import math
import numpy
#data structures
import Queue

#methods for comparison
from types import *


#A class that stores the x,y position of a node as well as its path cost and parent node
class Node(object):
    def __init__(self,x,y,free):
        self.x = x
        self.y = y
        self.g = 0
        self.f = 0
        self.parent = None

        def getX(self):
            return self.x
        
        def getY(self):
            return self.y

        def getG(self):
            return self.g

        def getParent(self):
            return parent

    #Overides the comparable function 
    #Allows the comparison of a node to tuples, lists and other nodes
    #@param a node
    #@type Node
    #@param a node or (x,y) tuple or list of node position
    #@type Node,tupile, or list
    def __cmp__(self,node,other):

        if type(other) is ListType or type(other) is TupleType:
            return node.getX() is other[0] and node.getY() is other[1]

        elif type(other) is Node:
            return node.getX() is other.getX() and node.y is other.y 

                    
# A* Search algorithim which computes the optimal path  and returns it as a stack
#TODO: Pass Data from the Costmap arrays into the Open List and closed list, this function assumes they already exist
class Astar(object):
    def __init__(self,ol,cl):

        #open and closed list arrays
        self.ol = ol
        self.cl = cl

        #open and closed list dictionaries
        self.oDict = {}
        self.cDict = {}        

        #Create an Open List priority Queue, 
        #items should be stored as (priority_number,data)
        openQ = Queue.PriorityQueue(maxsize=0)
         
        self.nodeGrid = [] #Array that will hold each node
        self.gridHeight = 84 + 6 #total number of Y spaces on the sim map grid
        self.gridWidth = 60 + 18 #total number of X spaces on the sim map grid

        
        #Return a node based on x,y coordinates found on stackoverflow
        #@param x coordinate
        #@type int
        #@param y coordinate
        #@type int
        #@return the node
        #@type Node
        def getNode(self,x,y):
             return self.nodeGrid[x * self.gridHeight + y]            

        #initialize a grid of corordinates 
        #@param None
        #@return None
        def grid(self):
            
            #Create a 2d grid array of nodes setting open nodes and closed nodes
            #TODO MAKE LIST A DICTIONARY
            for x in range(self.gridWidth):
                for y in range(self.gridHeight):
                    if(x,y) in closedList:
                        free = False
                        self.nodeGrid.append(Node(x,y,free))
                    else:                                                           
                        free = True
                        openListQ[x,y] = Node(x,y,free)
                        self.nodeGrid.append(Node(x,y,free))

            self.start = getNode(0,0) #(9,15)?
            self.goal= getNode(78,90) #(1,25)?

        #Convert the open and closed lists into dictionaries
        def convertToDict(self):

            for x in range(self.gridWidth):
                for y in range(self.gridHeight):
                    if getNode(x,y).free is True:
                        self.oDict[(x,y)] = getNode(x,y)
                    
                    elif getNode(x,y).free is False:
                        self.cDict[(x,y)] = getNode(x,y)
        
        #Convert to the Open List to Queue
        def convertToQueue(self):
            for x in range(len(self.ol)):
                self.openQ(self.f,self.ol[x])
                

                    
        #Compute the heuristic value of a cell in this case the Euclidean distance between the current node and the goal node
        #@param the current node
        #@type Node
        #@return the heuristic value
        #@type float
        def heuristic(self,node):        
            return math.sqrt(math.pow(self.goal[0] - node.getX(),2.0) + math.pow(self.goal[1] - node.getY(),2.0))

        #A function to update the state of a Node
        #@param the current node being checked
        #@type Node
        #@param the adjacent cell
        #@type Node
        #@return Nada
        def updateNode(self,node,adj):
            adj.g = cell.g + 1.0
            adj.h = heuristic(adj)
            adj.parent = node
            adj.f = adj.g + adj.h

        
        #Find the Neighboors of a given node
        #@param the Node
        #@type Node 
        #@return the neighboors of a node
        #@type a list of Nodes
        def getNeighbors(self,Node):
            nodes = []
            
            #if the cell is in the domain of the grid append the Node to the list of nodes

            #append the node to the left
            if(node.getX < self.gridWidth - 1):
                nodes.append(Node(x - 1, y))
            
            #append the node to the right
            if(node.getX < self.gridWidth - 1):
                nodes.append(Node(x + 1, y))

            #append the node below
            if(node.getY < self.gridHeight - 1):
                nodes.append(Node(x, y - 1))
            #append the node above
            if(node.getY < self.gridWidth - 1):
                nodes.append(Node(x, y + 1))

            return nodes

        #A function to return the path list based on the A* search algorithim
        #@param node
        #@return the path
        #@type list
        def getPath(self):
            pathList = []
            node = self.goal
            
            while node.parent is not self.start:
                node = node.parent
                pathList.append(node)
                
            print pathList
            return pathList
                
                
        #Run through the a* search algorithim and find the best path
        def search(self):

            #add starting node to the heapq open list Q
            self.openQ.put(self.f,self.start)

            for item in self.oDict:
                #pop the node from the p queue
                node = openQ.get()
            
                #add the current node to the closed list
                self.cDict((node.getX(),node.getY()) ,node)
            
                #if the ending node is found
                if node is self.goal:
                    self.getPath()
                    break
            
                #get the adjacent nodes for each node
                adjNodes = self.getNeighboors(node)
                
                #check each adjacent node 
                for n in adjNodes:
                    #if the node is free space and is not in the closed list
                    #check if the adj node should be added to list
                    if n.free and n not in self.cDict:
                        if n.g > node.g + 1:
                            self.updateNode(node,n)
                    else:
                        self.updateNode(node,n)
                        self.OpenQ.put(n.f,n)

    
                
            
def main():
 #   rospy.init_node('n')  #initialize node with the name n                                                                                          
#    rospy.Subscriber("base_scan",LaserScan,laserCallback) #node subscribes to base_scan topic which is of type LaserScan which invokes the laserCallback with the msg as first arg                                                                                                                                          
 #   rospy.Subscriber("path_seg", PathSegmentMsg, pathSegCallback) #subscribe to path_seg topic of type PathSegmentMsg using PathSegCallback                        

#   rospy.spin() 
    a = Astar((0,0),(5,0))


if __name__ == '__main__':
    main()
