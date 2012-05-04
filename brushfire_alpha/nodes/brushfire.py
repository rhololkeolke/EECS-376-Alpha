class BrushFire():
    def __init__(self, c1, c2, numCells, size=10, goal=None):
        self.globalc1 = c1
        self.globalc2 = c2
        self.numCells = numCells

        self.globalMap = createGrid(numCells)

        self.localMap = None
        self.size = size
        
        self.goal = goal
        self.robotPos = None

    def createGrid(self,numCells=50):
        '''
        This method uses the specified corners and the numCells to create a 2d array that will store the values used in the brushfire algorithm
        '''
        # empty list
        globalMap = list()
        
        for i in range(numCells):
            globalMap.append(list())
            for j in range(numCells):
                globalMap[i].append(0) # 0 is blank

        return globalMap
        
    def updateGlobalGrid(self, globalMap, c1, c2, obstacles):
        '''
        This method is responsible for taking in new obstacles
        and adding them to the global obstacle list
        '''
        for point in obstacles:
            try:
                # see if the point has a corresponding point in the grid
                gridPoint = self.transformMapToGrid(point, c1, c2, len(globalMap))
                globalMap[gridPoint[0]][gridPoint[1]] = -1
            except IndexError:
                # if the point isn't in the grid then ignore it
                pass
        return globalMap

    def extractLocal(self, numCells, x, y, size=10):
        '''
        This method takes in an x and y location for the robot
        It then takes a subsection of the global that is 
        size wide and size high.
        
        This local map will be used by the brushfire algorithm.
        '''

        # Get the robot's current position
        robot = transformMapToGrid((x,y), self.c1, self.c2, self.numCells)
        
        # will store the local map
        localMap = list()

        # check and see if the local grid desired would be out of
        # bounds when centered on the robot
        # if it is then those out of bound spaces will need to
        # be filled with 1's to denote obstacles
        # this way the robot does not accidently try and steer to
        # these nodes
        rowUnderFlow = robot[0] - size
        rowOverFlow = numCells - robot[0] - size
        colUnderFlow = robot[1] - size
        colOverFlow = numCells - robot[1] - size

        # might need to be <=
        if(rowOverFlow < 0):
            # for each extra row needed
            for i in range(-rowOverFlow):
                # create a sub array filled with ones
                temp = list()
                for j in range(numCells):
                    temp.append(1)
                localMap.append(temp)
        
        localMap = localMap + globalMap[max(robot[0]-size,0):min(robot[0]+size,numCells)]
                    

        localMap = 
        
        # go through and slice out the desired columns
        for i,row in enumerate(localMap):
            localMap[i] = row[max(robot[1]-size,0):min(robot[1]+size,numCells)]

        return (robot,localMap)

    def transformGridToMap(self, point, c1, c2, numCells):
        if(point[0] < 0 or point[0] >= numCells):
            raise IndexError
        if(point[1] < 0 or point[1] >= numCells):
            raise IndexError

        # width and height of grid cells
        xStep = abs(c1[0] - c2[0])/float(numCells)
        yStep = abs(c1[1] - c2[1])/float(numCells)

        x = xStep*point[0] + min(c1[0],c2[0])
        y = yStep*point[1] + min(c1[1],c2[1])

        return (x,y)
        
    def transformMapToGrid(self, point, c1, c2, numCells):
        # width and height of grid cells
        xStep = float(abs(c1[0] - c2[0]))/numCells
        yStep = float(abs(c1[1] - c2[1]))/numCells

        # translate the goal point to grid space
        xIndex = int((point[0]-min(c1[0],c2[0]))/xStep)
        yIndex = int((point[1]-min(c1[1],c2[1]))/yStep)

        # make sure the indices of the point are within
        # the specified grid dimensions
        if(xIndex >= numCells or xIndex < 0):
            raise IndexError
        if(yIndex >= numCells or yIndex < 0):
            raise IndexError
        
        return (xIndex,yIndex)

    def brushfire():
        pass
    def computePath():
        pass
    def updateGoal():
        pass
