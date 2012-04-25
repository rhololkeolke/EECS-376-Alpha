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

#data structures
import Queue

#misc
from types import *
import random



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

        self.convertToDict()

        #Create an Open List priority Queue, 
        #items should be stored as (data,priority_number)
        self.openQ = Queue.PriorityQueue(maxsize=0)
         
        self.nodeGrid = [] #Array that will hold each node
        self.gridHeight = 84 + 6 #total number of Y spaces on the sim map grid
        self.gridWidth = 60 + 18 #total number of X spaces on the sim map grid

        self.start = self.getNode(0,0) #(9,15)?
        self.goal= self.getNode(5,5) #(1,25),(78,90)?

#        print self.start
 #       print self.goal

        
    #initialize a grid of corordinates 
    #@param None
    #@return None
    def grid(self):

        print "Initializing Grid"
        

        #Create a 2d grid array of nodes setting open nodes and closed nodes
        for x in range(self.gridWidth):
            for y in range(self.gridHeight):
                if(x,y) in self.cDict:
                    free = False
                    self.nodeGrid.append(Node(x,y,free))
                else:                                                           
                    free = True
                    self.oDict[x,y] = Node(x,y,free)
                    self.nodeGrid.append(Node(x,y,free))

        print "Grid Initialization Complete"


    #Convert the open and closed lists into dictionaries
    def convertToDict(self):
        print "Converting Dictionaries"

         #Add each open and closed list to a dictionary with the (x,y) coord as the key and the Node at that (x,y) as the value
        for coord in self.ol:
            self.oDict[coord] = Node(coord[0],coord[1],free=True)
            
        for coord in self.cl:
            self.cDict[coord] = Node(coord[0],coord[1],free=False)

        
        print "Dictionary Conversion Completed"

  


    #Convert to the Open List to Queue
    def convertToQueue(self):
        print "Doing Queue Conversion"
        
        for x in range(len(self.ol)):
            self.openQ(self.f,self.ol[x])

        print "Finished with Queue Conversion"

    #Return a node based on x,y coordinates 
    #@param x coordinate
    #@type int
    #@param y coordinate
    #@type int
    #@return the node
    #@type Node
    #TODO: FIND A FASTER WAY TO RETURN A NODE
    def getNode(self,x,y):

        tup = (x,y)
        print "Getting the node: %s" % str(tup)        
        if (x,y) in self.oDict:

            return self.oDict[(x,y)]        

        elif (x,y) in self.cDict:
            
            return self.cDict[(x,y)]

        else:
            print "The %s is not in the open list or closed list" % str(tup)



        print "Got the node"


    #Compute the heuristic value of a cell in this case the Euclidean distance between the current node and the goal node
    #@param the current node
    #@type Node
    #@return the heuristic value
   #@type float
    def heuristic(self,node):        
        
        print "Computing h(x)"
        
        return math.sqrt(math.pow(self.goal.getX() - node.getX(),2.0) + math.pow(self.goal.getY() - node.getY(),2.0))




    #A function to update the state of a Node
    #@param the current node being checked
    #@type Node
    #@param the adjacent cell
    #@type Node
    #@return Nada
    def updateNode(self,node,adj):
        
        print "Updating a Node's state"

#        adj.g = node.g + 1.0
        adj.h = self.heuristic(adj)

        adj.parent = node

        adj.f = adj.g + adj.h


        print "State Updated"
 
        
    #Find the Neighboors of a given node
    #@param the Node
    #@type Node 
    #@return the neighboors of a node
    #@type a list of Nodes
    def getNeighboors(self,node):
        
        tup = (node.getX(),node.getY())
        print "Finding Neighboors of: %s" % str(tup)

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

            node = self.getNode(node.getX(), node.getY() + 1)
            
            if type(node) is not NoneType:
                nodes.append(node)

        print "Got the neighboors"
        
        for n in nodes:
            print (n.getX(),n.getY())
        

        return nodes
        
    #A function to return the path list based on the A* search algorithim
    #@param node 
    #@return the path
    #@type list
    def getPath(self):
        
        print "Printing the generated Path"

        pathList = []
        node = self.goal
        
        while node.parent is not self.start:
            node = node.parent
            pathList.append(node)
            
        print pathList
        return pathList
                
                
    #Run through the a* search algorithim and find the best path
    def search(self):

        print "Starting a Search"

        #add starting node to the heapq open list Q
        self.openQ.put(self.start,self.start.f)
        
        for item in self.oDict.items():
            
            #pop the node from the p queue
            node = self.openQ.get()
            

            #add the current node to the closed list
            self.cDict[(node.getX(),node.getY())] = node
            print "Checking Node"
            print node.getX() 
            print node.getY()
            
            #if the ending node is found
            if node is self.goal:
                print "I think I found the goal"
                self.getPath()
                break
            
            
            
            #get the adjacent nodes for each node
            adjNodes = self.getNeighboors(node)
            print len(adjNodes)
            
            #check each adjacent node 
            for n in adjNodes:
                print "Inside the adjacent Nodes loop of self.search"
                
                #update the cost 
                n.g = node.g + 1
                
                #if the node is free space and is not in the closed list
                #check if the adj node should be added to list
                
                if n.getFree() is True and (n.getX(),n.getY())  not in self.cDict:
                    
                    print '%s  is free and not in the closed list' % str((n.getX(),n.getY()))
                    if (n.getX(),n.getY() in self.oDict):
                                             
                        if n.getG() is node.getG() + 1:
                            print "Yea, I did determine that n.g > node.g + 1"
                            self.updateNode(node,n)

                        else:
                            print "How am I supposed to update anything when g = g?"
                    else:
                        print "Neighboor wasn't free"    
                        self.updateNode(node,n)
                        self.openQ.put(n,n.f)

        print "closing for each loop"

                        
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

    #generate grid points equivalent to that of the map grid
    for x in range(gridWidth):
        for y in range(gridHeight):
            gridPoints.append((x,y))
    
    for p in gridPoints:
        
        if random.random() > 0.9:
            cl.append(p)

        else:
            ol.append(p)


    coList.append(ol)
    coList.append(cl)

    print "Done generating Test Points"
    
    return coList
    

def main():
 #   rospy.init_node('n')  #initialize node with the name n                                                                                          
#    rospy.Subscriber("base_scan",LaserScan,laserCallback) #node subscribes to base_scan topic which is of type LaserScan which invokes the laserCallback with the msg as first arg                                                                                                                                          
 #   rospy.Subscriber("path_seg", PathSegmentMsg, pathSegCallback) #subscribe to path_seg topic of type PathSegmentMsg using PathSegCallback                        

#   rospy.spin() 

    #Initialize Test Data
    olCl = test()
    ol = olCl[0] #open list
    cl = olCl[1] #closed list
 
    a = Astar(ol,cl)

#    n = Node(0,0,free=True)
 #   print  n.getG()
    a.search()
 #   nebs = a.getNeighboors(Node(0,0,free=True))

#    a.updateNode(a.getNode(0,0),nebs[1])
     
if __name__ == '__main__':
    main()
