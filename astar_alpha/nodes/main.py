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

#ros msg data types
from msg_alpha.msg._PointList import PointList as Path_Points
from geometry_msgs.msg._Point import Point as PointMsg

#mathematics
import math

#data structures
import Queue

#misc
from types import *
import random

pose = None
closedList = None


def closedListCallback(data):
    global closedList
    closedList = data
    

def poseCallback(poseData):
    global pose
    pose = poseData




#A class that stores the x,y position of a node as well as its path cost and parent node
class Node(object):
    def __init__(self,x,y,free):
        self.x = x
        self.y = y
        self.free = free
        self.g = 0
        self.h = 0
        self.f = 0
        self.parent = None

    def getX(self):
        return self.x
     
    def getY(self):
        return self.y
    
    def getFree(self):
        return self.free

    def getG(self):
        return self.g
    
    def getParent(self):
        return parent
    
        
                    
# A* Search algorithim which computes the optimal path based on a list of closed points
#TODO: Pass Data from the Costmap arrays into closed list, this function assumes they already exist
class Astar(object):
    def __init__(self,cl):

        self.cl = cl #closed list
        self.cDict = {} #closed list dictionary
        self.convertToDict() #convert closed list into dictionary
        #self.transformToGrid()
        self.oDict = {} 
        self.completeDict = {} #dictionary storing all nodes for getNode() function
        self.start = None
        self.goal = None
        
 

        #Create an Open List priority Queue, 
        #items should be stored as (data,priority_number)
        self.openQ = Queue.PriorityQueue(maxsize=0)
         
        self.nodeGrid = [] #Array that will hold each node
        self.gridHeight = 90 #total number of Y spaces on the sim map grid
        self.gridWidth = 78 #total number of X spaces on the sim map grid

        self.grid()


    #Translate the costmap coordinates from /map frame to Astar grid frame
    def transformToGrid(self,closedList):
        cl = []
        for p in self.cl:
            p = (p[0] + 100, p[1] + 100)
            cl.append(p)

        self.cl = cl



        
    #initialize a grid of corordinates 
    #@param None
    #@return None
    def grid(self):
        print "Initializing a grid"

        #Create a 2d grid array of nodes setting open nodes and closed nodes
        for x in range(self.gridWidth + 1):
            for y in range(self.gridHeight + 1):
                if(x,y) in self.cDict:
                    free = False
                    node = Node(x,y,free)
                    self.nodeGrid.append(node)
                    self.completeDict[(x,y)] = node
                else:                                                           
                    free = True
                    node = Node(x,y,free)
                    self.nodeGrid.append(node)
                    self.completeDict[(x,y)] = node

    #A function to convert the closed list into a dictionary.
    def convertToDict(self):

        #Add each open and closed list to a dictionary with the (x,y) coord as the key and the Node at that (x,y) as the value
        for coord in self.cl:
            self.cDict[coord] = Node(coord[0],coord[1],free=False)

  
    #Return a node based on x,y coordinates 
    #@param x coordinate
    #@type int
    #@param y coordinate
    #@type int
    #@return the node
    #@type Node
    def getNode(self,x,y):

        if (x,y) in self.completeDict:

            return self.completeDict[(x,y)]
        else:
            pass

        
        
    #Compute the heuristic value of a cell in this case the Euclidean distance between the current node and the goal node
    #@param the current node
    #@type Node
    #@return the heuristic value
   #@type float
    def heuristic(self,node):        
        
        return math.sqrt(math.pow(self.goal.getX() - node.getX(),2.0) + math.pow(self.goal.getY() - node.getY(),2.0))


    #A function to update the state of a Node
    #@param the current node being checked
    #@type Node
    #@param the adjacent cell
    #@type Node
    #@return Nada
    def updateNode(self,node,adj):

        adj.g = node.g + 1.0
        adj.h = self.heuristic(adj)
        adj.parent = node
        adj.f = adj.g + adj.h

        
    #Find the Neighboors of a given node
    #@param the Node
    #@type Node 
    #@return the neighboors of a node
    #@type a list of Nodes
    def getNeighboors(self,node):
        
       nodes = []
          
        #if the cell is in the domain of the grid append the Node to the list of nodes
        #append the node to the left
       if(node.getX() < self.gridWidth - 1):
           if type(self.getNode(node.getX() - 1, node.getY())) is not NoneType:
               
               nodes.append(self.getNode(node.getX() - 1, node.getY()))
               
        #append the node to the right
       if node.getX() < self.gridWidth - 1:

           if type(self.getNode(node.getX() + 1, node.getY())) is not NoneType:

                       nodes.append(self.getNode(node.getX() + 1, node.getY()))

       #append the node below
       if node.getY() < self.gridHeight - 1:
           if type(self.getNode(node.getX(),node.getY() - 1)) is not NoneType:

               nodes.append((self.getNode(node.getX(),node.getY() - 1)))


       #append the node above
       if(node.getY() < self.gridHeight - 1):
            
           if type(self.getNode(node.getX(), node.getY() + 1)) is not NoneType:

               nodes.append(self.getNode(node.getX(), node.getY() + 1))


       #append the lower left diagnol
       if node.getX() < self.gridWidth - 1 and node.getY() < self.gridHeight -1:

           if type(self.getNode(node.getX() -1 , node.getY() - 1)) is not NoneType:
                
               nodes.append(self.getNode(node.getX() -1 , node.getY() - 1))

       #append the lower right
       if type(node) is not NoneType:
            
           if type(self.getNode(node.getX() + 1 , node.getY() -1)) is not NoneType:
               
               nodes.append(self.getNode(node.getX() + 1 , node.getY() -1))

        #upper left
       if node.getX() < self.gridWidth - 1 and node.getY() < self.gridHeight -1:
            
           if type(self.getNode(node.getX() -1 , node.getY() + 1)) is not NoneType:
               
               nodes.append(self.getNode(node.getX() -1 , node.getY() + 1))

       #upper right
       if node.getX() < self.gridWidth - 1 and node.getY() < self.gridHeight - 1:
            
           if type(self.getNode(node.getX() + 1,node.getY() + 1)) is not NoneType:

               nodes.append(self.getNode(node.getX() + 1,node.getY() + 1))
                

       return nodes
        
                
                
    #Run through the a* search algorithim and find the best path
    #TODO: Include case when there is not available path
    def search(self):
        

        start = self.getNode(9,15) #(9,15)?
        goal= self.getNode(78,89) #(1,25),(78,90)?

        #To ensure that heuristic() can access the goal point
        self.start = start
        self.goal = goal


        #add starting node to the open list Q
        self.openQ.put_nowait((start.f,start))

        #if self.openQ.empty() is True:
         #   print "There is no Path from %f %f to %f %f" %(start.getX(),start.getY()) %(goal.getX(),goal.getY())

        while self.openQ.empty() is False:
            
            #pop the node from the p queue
            nodeQ = self.openQ.get_nowait() #returns the Node and its priority
            node = nodeQ[1] #get the node out of the tupile returned from .get_nowait()
            

            #add the current node to the closed list
            self.cDict[(node.getX(),node.getY())] = node
            
            
            #if the ending node is found
            if node is goal:

                self.getPath()
                break

            
            #get the adjacent nodes for each node
            adjNodes = self.getNeighboors(node)

            #check each adjacent node 
            for n in adjNodes:
                 
                #if the node is in the closed dictionary ignore it
                if (n.getX(),n.getY()) in self.cDict:
                    pass

                
                #if the node is not in the open dictionary add it to the open dictionary and open queue
                #update the costs of the node
                elif ((n.getX(),n.getY())) not in self.oDict:

                    self.updateNode(node,n)
                    
                    self.openQ.put_nowait((n.f,n)) 
                    self.oDict[(n.getX(),n.getY())] = n


                #if the node is already in the open list see if there is a better path    
                elif((n.getX(),n.getY())) in self.oDict:
                  

                    #if the adjacent node has a lower cost value then change its parent to the current node
                    if n.getG() < node.getG():
                        self.updateNode(node,n)      



    #A function to return the path list based on the A* search algorithim
    #@param node 
    #@return the path
    #@type list
    def getPath(self):

        pathList = []
        node = self.goal
        
        #find the path to the node by tracing back the node's parent pointers
        while node.parent is not self.start:
            node = node.parent 
            
            pathList.append((node.getX(),node.getY()))
            
        print pathList
#        pub.publish(pathList)
#        return pathList
        self.transformPath(pathList)


    def transformToMap(self,pathList):

        pathListPub = rospy.Publisher('point_list', Path_Points)     #Data should be published to the obstacles topics using the Obstacles message type                            
        pathData = Path_Points() #initalize an Obstacle message                   
        
        transList = [] #list transformed into map coordinates

        for p in pathList:
            transList.append((p[0] + -100, p[1] + -100, 0))
        transList.append(pose)
        transList.reverse()
        transList.append(goal)

        pathData.path_points = transList
        pathListPub.publish(pathData)
        print transList
        


                        
#Generate a bunch of random x,y coordinates)
#@return  open and closed list coords
#list                    
def test():

    print "Generating Test Points"
    
    gridWidth = 78
    gridHeight = 90
    gridPoints = [] #all points

    ol = [] #open 
    cl = [] #closed
    coList = [] #open and closed combined

    print "Outside to test point loop"
    #generate grid points equivalent to that of the map grid
    for x in range(gridWidth + 1):
        for y in range(gridHeight + 1):
            gridPoints.append((x,y))
    
    for p in gridPoints:
        
        if random.random() > 0.9:
            cl.append(p)

        else:
            ol.append(p)


    coList.append(ol)
    coList.append(cl)

    
    return coList
    

def main():

    global closedList
    rospy.init_node('n')  #initialize node with the name n                                                                                          

    pathPointPub = rospy.Publisher('point_list',Path_Points) #publish to the "point_list" topic using the "Path_Points" message
    pathData = Path_Points()
    rospy.Subscriber('costmap_alpha/costmap/inflatedobstacles',PointMsg,closedListCallback)


    if closedList == None:
        print "A* has no closed points"
    #Initialize Test Data

#    olCl = test()
 #   cl = olCl[1] #closed list
  #  ol = olCl[0]
    while not rospy.is_shutdown():
#            rospy.spin() 
        a = Astar(closedList)
        a.search()

     
if __name__ == '__main__':
    main()
