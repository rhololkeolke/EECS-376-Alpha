class Astar():
    def __init__(self, corner1, corner2, numCells):
        '''
        Constructor for Astar class.
        
        corner1 and corner2 are tuples of the form (x,y) where x and y are coordinates
        numCells is an integer
        path is a list of geometry_msg Point messages
        '''
        
        self.c1 = corner1
        self.c2 = corner2
        self.numCells = numCells
        
        # stores the latest computed path
        self.path = []
        self.__pathDict = dict()

        # should be a tuple specifying the starting coordinates in map frame
        self.start = None

        # should be a tuple specifying the ending coordinates in map frame
        self.goal = None
        
        self.grid = self.createGrid()

    def createGrid(self):
        '''
        This method uses the specified corners and the numCells to
        create a 2d array that will store the values used in the astar
        search
        '''
        # empty list
        mapArray = list()

        for i in range(self.numCells):
            mapArray.append(list())
            for j in range(self.numCells):
                mapArray[i].append(0) # 0 is blank, 1 is path, -1 is obstacle

        return mapArray

    def updateGoal(self,goal,recompute=True):
        '''
        Will update the saved goal location. If recompute is left
        with the default value of True the path will be recomputed
        '''
        self.goal = goal
        
        if recompute:
            self.computePath() # compute the path using the new goal and the saved start point
            return True
        else:
            return False

    def updateClosedList(self,closedList,recompute=True):
        '''
        Will change the values in the closed list
        If any of the points in the closed list are on a path point
        Then the path will be recomputed, unless recompute is set
        to False.

        This method will return True if a new path was computed
        and false otherwise
        '''

        # update the grid with all of the closed points
        (conflict, self.grid) = self.populateGrid(closedList)
        
        # if there was a conflict and recompute was set to true then call the 
        # computePath method
        if conflict and recompute:
            self.computePath()
            return True
        else:
            return False

    def populateGrid(self, closedList):
        '''
        Given a list of closed points this function will
        fill in the new grid with the path and check if a
        new path needs to be calculated. If a new path needs
        to be calculated then the populated grid
        will be used in the calculations. Otherwise the populated
        grid will be discarded
        '''
        newGrid = self.createGrid()

        conflict = False
        # fill in the squares in the grid that are included in the current path
        for point in closedList:
            try:
                # see if the point has a corresponding point in the grid
                gridPoint = self.transformMapToGrid(point)
            except Exception:
                # if the point isn't in the grid then it doesn't matter
                continue

            if(self.__pathDict.get(gridPoint,False)):
                conflict = True

            newGrid[gridPoint[0]][gridPoint[1]] = -1

        return (conflict, newGrid)

    def transformMapToGrid(self, point):
        from math import floor
        # width and height of grid cells
        xStep = float(abs(self.c1[0] - self.c2[0]))/self.numCells
        yStep = float(abs(self.c1[1] - self.c2[1]))/self.numCells

        # translate the goal point to the grid space
        xIndex = int(floor((point[0]-min(self.c1[0],self.c2[0]))/xStep))
        yIndex = int(floor((point[1]-min(self.c1[1],self.c2[1]))/yStep))

        # make sure the indices of the goal are within the 
        # specified grid
        if(xIndex >= self.numCells or xIndex < 0):
            raise IndexError
        if(yIndex >= self.numCells or yIndex < 0):
            raise IndexError

        return (xIndex,yIndex)

    def transformGridToMap(self, point):        
        if(point[0] < 0 or point[0] >= self.numCells):
            raise IndexError
        if(point[1] < 0 or point[1] >= self.numCells):
            raise IndexError
        
        # width and height of grid cells
        xStep = abs(self.c1[0] - self.c2[0])/float(self.numCells)
        yStep = abs(self.c1[1] - self.c2[1])/float(self.numCells)

        x = xStep*point[0] + min(self.c1[0],self.c2[0])
        y = yStep*point[1] + min(self.c1[1],self.c2[1])

        return (x,y)
    
    def computePath(self, start=None, goal=None):
        '''
        This method is responsible for computing an optimum path
        from the specified start point to the goal point.

        start is a tuple of the form (x,y) where x and y are coordinates
        '''
        from priority_dict import priority_dict as PriorityDict
        from space import Space
        
        # see if there is a start point
        # if there isn't then just return
        # also save the start point if it is new
        if start is None:
            if(self.start is None):
                return
            else:
                start = self.start
        else:
            self.start = start

        # see if there is a goal point
        # if there isn't then just return
        # also save the goal point
        # if it is new
        if goal is None:
            if(self.goal is None):
                return
            else:
                goal = self.goal
        else:
            self.goal = goal

        # copy the saved closedList
        # probably a more efficient way of doing this
        closedList = list()
        for i,row in enumerate(self.grid):
            closedList.append(list())
            for j,cell in enumerate(self.grid[i]):
                closedList[i].append(cell)

        try:
            goal = self.transformMapToGrid(self.goal)
            start = self.transformMapToGrid(start)
        except Exception:
            return

        # starting node in the search tree
        root = Space((start[0],start[1]),(goal[0],goal[1]))

        # create the openList
        openList = PriorityDict()

        # This will be filled with the goal state when
        # the goal state is expanded
        # if the goal is never found then this will remain
        # None
        goalSpace = None

        openList[root.point] = root

        while(len(openList) > 0):
            currSpace = openList.pop_smallest()
            closedList[currSpace.point[0]][currSpace.point[1]] = -1

            # see if this is the goal space
            if(currSpace.point == goal):
                # if it is then save the space
                # and return
                goalSpace = currSpace
                break

            neighbors = self.getNeighbors(currSpace.point)
            
            # for each of the potential new points
            for point in neighbors:
                # make sure the point isn't already closed
                if(closedList[point[0]][point[1]] != -1):
                    if point in openList:
                        # update the cost if necessary
                        if(openList[point].g > currSpace.g+1):
                            tempSpace = openList[point]
                            tempSpace.g = currSpace.g+1
                            openList[point] = tempSpace
                    else:
                        # add the point to the openlist
                        openList[point] = Space(point,goal,currSpace)
        
        if goalSpace is None:
            # no path could be found
            # clear the old path
            self.path = []
            self.__pathDict = dict()
            return

        reversePath = list()
        
        # start by adding the goal to the list
        reversePath.append(goalSpace.point)
        
        currSpace = goalSpace
        # travel up the tree until the root is reached
        while currSpace.parent is not None:
            currSpace = currSpace.parent
            reversePath.append(currSpace.point)

        # now flip the path so that it is in the correct order
        self.path = reversePath[::-1] # for all elements in reversePath in steps of negative 1 from the end

        # put the path in dictionary form so that the updatedClosedList can take advantage of the path information    
        # there are better more pythonic ways of doing this, but this should work
        self.__pathDict = dict()
        for point in self.path:
            self.__pathDict[point] = True

        # transform all of the points back into the map frame
        for i,point in enumerate(self.path):
            self.path[i] = self.transformGridToMap(point)
                    

    def getNeighbors(self, point):
        '''
        Given a point in grid space find all of viable neighbors
        Return this as a list of tuples
        '''
        neighbors = list()
        
        # up
        x = point[0]
        y = point[1] + 1
        if(y < self.numCells and y >=0 and x < self.numCells and x >= 0):
            neighbors.append((x,y))

        # up right
        x = point[0] + 1
        y = point[1] + 1
        if(y < self.numCells and y >=0 and x < self.numCells and x >= 0):
            neighbors.append((x,y))

        # right
        x = point[0] + 1
        y = point[1]
        if(y < self.numCells and y >=0 and x < self.numCells and x >= 0):
            neighbors.append((x,y))

        # down right
        x = point[0] + 1
        y = point[1] - 1
        if(y < self.numCells and y >=0 and x <self.numCells and x >= 0):
            neighbors.append((x,y))

        # down
        x = point[0]
        y = point[1] - 1
        if(y < self.numCells and y >=0 and x < self.numCells and x >= 0):
            neighbors.append((x,y))

        # down left
        x = point[0] - 1
        y = point[1] - 1
        if(y < self.numCells and y >=0 and x < self.numCells and x >= 0):
            neighbors.append((x,y))

        # left
        x = point[0] - 1
        y = point[1]
        if(y < self.numCells and y >=0 and x < self.numCells and x >= 0):
            neighbors.append((x,y))

        # up left
        x = point[0] - 1
        y = point[1] + 1
        if(y < self.numCells and y >=0 and x < self.numCells and x >= 0):
            neighbors.append((x,y))

        return neighbors
