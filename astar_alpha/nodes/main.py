#!/usr/bin/env python                                                                                                                                                 

#ros imports
import roslib; roslib.load_manifest('astar_alpha')
import rospy
#ros package imports


#mathematics
import math
import numpy
#data structures
import heapq


#A class that stores the x,y position of a node as well as its path cost and heurisitc value
class Node(object):
    def __init__(self,x,y,free):
        self.x  = x
        self.y = y
        self.g = 0
        self.h = math.sqrt(self.x)
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

        print "Initializing a node"
                    

# A* Search algorithim which computes the optimal path  and returns it as a stack
class Astar(object):
    def __init__(self):
        self.openList = []
        heapq.heapify(self.openList)
        self.closedList = []
        self.nodes = []
        self.gridHeight = 84 + 6 #total number of Y spaces on the sim map grid
        self.gridWidth = 60 + 18 #total number of X spaces on the sim map grid

        print "Initializing the an a star object"

        
        #initialize a grid of corordinates 
        #@param None
        #@return None
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

            self.start = (0,0) #(9,15)?
            self.goal= (78,90) #(1,25)?

        #Compute the heuristic value of a cell in this case the Euclidean distance between the current node and the goal node
        #@param the current node
        #@type Node
        #@return the heuristic value
        #@type float
        def heuristic(self,node):        
            return math.sqrt(math.pow(self.goal(0) - node.getX(),2.0) + math.pow(self.goal(1) - node.getY(),2.0))

        
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
            
            #if the cell is in the domain of the grid append the Node directly to the left to the array of nodes
            if(node.getX < self.gridWidth - 1):
                nodes.append(Node(x - 1, y))
            
            #append the cell to the right
            if(node.getX < self.gridWidth - 1):
                nodes.append(Node(x + 1, y))

            #append the cell below
            if(node.getY < self.gridHeight - 1):
                nodes.append(Node(x, y - 1))
            #append the cell above
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
            #add starting cell to the heapq open list
            heapq.push(self.op(self.start.f, self.start))

            while len(self.op):
                #pop the node from the p queue
                node = heapq.heapop(self.op)
            
                #add the current node to the closed list
                self.cl.append(node)
            
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
                    if n.free and n not in self.closedList:
                        if n.g > node.g + 1:
                            self.updateNode(node,n)
                    else:
                        self.updateNode(node,n)
                        heap.heapush(self.op(c.f,c))
    
                
            
def main():
 #   rospy.init_node('n')  #initialize node with the name n                                                                                          
#    rospy.Subscriber("base_scan",LaserScan,laserCallback) #node subscribes to base_scan topic which is of type LaserScan which invokes the laserCallback with the msg as first arg                                                                                                                                          
 #   rospy.Subscriber("path_seg", PathSegmentMsg, pathSegCallback) #subscribe to path_seg topic of type PathSegmentMsg using PathSegCallback                        

#    rospy.spin() 
    Astar()
    print "In the main function"

if __name__ == '__main__':
    main()
