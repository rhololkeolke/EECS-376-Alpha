class BrushFire():
    import math
    def __init__(self, c1, c2, numCells, size=10, goal=None):
        self.globalc1 = c1
        self.globalc2 = c2
        self.numCells = numCells

        self.globalMap = self.createGrid(numCells)

        self.localMap = None
        self.localx = None
        self.localy = None
        self.size = size
        
        self.goal = goal
        self.robotPos = None

        self.pathList = None
        self.localPathList = None

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
        
    def updateGlobalGrid(self, obstacles):
        '''
        This method is responsible for taking in new obstacles
        and adding them to the global obstacle list
        '''
        globalMap = self.globalMap
        numCells = self.numCells

        for point in obstacles:
            try:
                # see if the point has a corresponding point in the grid
                gridPoint = self.transformMapToGrid(point)
                globalMap[gridPoint[0]][gridPoint[1]] = 1
            except IndexError:
                # if the point isn't in the grid then ignore it
                pass

        self.globalMap = globalMap

    def extractLocal(self, x, y):
        '''
        This method takes in an x and y location for the robot
        It then takes a subsection of the global that is 
        size wide and size high.
        
        This local map will be used by the brushfire algorithm.
        '''

        # Get the robot's current position in the global grid
        try:
            self.robot = self.transformMapToGrid((x,y))
        except IndexError:
            print "Error robot apparently outside of grid"

        self.localx = (self.robot[0]-self.size-1,self.robot[0]+self.size)
        self.localy = (self.robot[1]-self.size-1,self.robot[1]+self.size)
        
        # will store the local map
        localMap = list()

        for i in range(self.localx[0],self.localx[1]):
            localMap.append(list())
            for j in range(self.localy[0],self.localy[1]):
                if(i >= 0 and i < self.numCells and j >= 0 and j < self.numCells):
                    localMap[-1].append(self.globalMap[i][j])
                else:
                    localMap[-1].append(1)

        self.localMap = localMap

    def transformGridToMap(self, point):
        c1 = self.globalc1
        c2 = self.globalc2
        numCells = self.numCells
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

    def transformMapToGrid(self, point):
        c1 = self.globalc1
        c2 = self.globalc2
        numCells = self.numCells
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

    def transformLocalToGlobal(self, point):
        '''
        Transform a point in the localMap to the corresponding
        point in the globalMap
        '''
        return (point[0]+self.localx[0],point[1]+self.localy[0])

    def getNeighbors(self, point):
        '''
        given a point in grid space find all of viable neighbors
        return this as a list of tuples
        '''

        height = 2*self.size+1
        neighbors = list()

        # up
        x = point[0]
        y = point[1] + 1
        if(y < height and y >=0 and x < height and x >= 0):
            neighbors.append((x,y))

        # up right
        x = point[0] + 1
        y = point[1] + 1
        if(y < height and y >=0 and x < height and x >= 0):
            neighbors.append((x,y))

        # right
        x = point[0] + 1
        y = point[1]
        if(y < height and y >=0 and x < height and x >= 0):
            neighbors.append((x,y))

        # down right
        x = point[0] + 1
        y = point[1] - 1
        if(y < height and y >=0 and x < height and x >= 0):
            neighbors.append((x,y))

        # down
        x = point[0]
        y = point[1] - 1
        if(y < height and y >=0 and x < height and x >= 0):
            neighbors.append((x,y))

        # down left
        x = point[0] - 1
        y = point[1] - 1
        if(y < height and y >=0 and x < height and x >= 0):
            neighbors.append((x,y))

        # left
        x = point[0] - 1
        y = point[1]
        if(y < height and y >=0 and x < height and x >= 0):
            neighbors.append((x,y))

        # up left
        x = point[0] - 1
        y = point[1] + 1
        if(y < height and y >=0 and x < height and x >= 0):
            neighbors.append((x,y))

        return neighbors

    def brushfire(self):
        '''
        Given a square grid of obstacles runs brushfire and returns grid
        '''
        localMap = self.localMap
        seenZero = True
        # if there are no zeros seen in a loop, we are done with brushfire
        value = 1
        while seenZero and value <= 2*self.size+1:
            seenZero = False
            for r,row in enumerate(localMap):
                for c,cell in enumerate(row):
                    if cell == value:
                        seenZero = True
                        neighbors = self.getNeighbors((r,c))
                        for pr,pc in neighbors:
                            if localMap[pr][pc] == 0:
                                localMap[pr][pc] += value + 1
            value +=1
        self.localMap = localMap

    def computePath(self):
        '''
        take grid of points passed through brushfire and returns list of points
        to follow
        '''
        import math
        goal = self.goal
        localMap = self.localMap
        gridGoal = self.transformMapToGrid(goal)
        center = len(localMap)//2
        robot = (center,center)
        robotGrid = self.transformLocalToGlobal(robot)
        robotDist = math.sqrt(pow((gridGoal[0] - robotGrid[0]),2) +
                pow((gridGoal[1] - robotGrid[1]),2))
        minDist = None
        highestPoint = None
        pathList = []
        lastPoint = robot
        count = 0
        while not (lastPoint[0] == 0 or lastPoint[0] == 2*self.size or lastPoint[1] == 0 or lastPoint[1] == 2*self.size) and count < 10:
            for point in self.getNeighbors(lastPoint):
                gridPoint = self.transformLocalToGlobal(point)
                pointDist = math.sqrt(pow((gridGoal[0] - gridPoint[0]),2) +
                    pow((gridGoal[1] - gridPoint[1]),2))
                if highestPoint is None:
                    minDist = pointDist
                    highestPoint = point
                elif localMap[point[0]][point[1]] == localMap[highestPoint[0]][highestPoint[1]]:
                    if pointDist < minDist:
                        minDist = pointDist
                        highestPoint = point
                elif localMap[point[0]][point[1]] > localMap[highestPoint[0]][highestPoint[1]]:
                    highestPoint = point
                    minDist = pointDist
            if(highestPoint == lastPoint):
                break
            pathList.append(highestPoint)
            lastPoint = highestPoint
            count += 1
    
        self.localPathList = pathList
        #print pathList
        for i,point in enumerate(pathList):
            pathList[i] = self.transformLocalToGlobal(point)
            pathList[i] = self.transformGridToMap(pathList[i])

        print pathList
        
        self.pathList = pathList

    def updateGoal(self, goal):
        '''
        Will save the given goal point to the class
        '''
        self.goal = goal

    def __str__(self):
        '''
        Displays the current local map as ASCII art
        '''

        class bcolors:
            HEADER = '\033[95m'
            OKBLUE = '\033[94m'
            OKGREEN = '\033[92m'
            WARNING = '\033[93m'
            FAIL = '\033[91m'
            ENDC = '\033[0m'

            def disable(self):
                self.HEADER = ''
                self.OKBLUE = ''
                self.OKGREEN = ''
                self.WARNING = ''
                self.FAIL = ''
                self.ENDC = ''


        printSpacedCharacter = self.printSpacedCharacter
        numSpaces = 4

        if(self.localMap is None):
            return 'None'

        display = ' '*numSpaces
        display += '|'

        # print the column headings
        for i in range(2*self.size+1):
            display += printSpacedCharacter(i,numSpaces)

        display = display + '\n'

        display += '-'*(numSpaces+1)
        # add a horizontal rule
        for i in range(2*self.size+1):
            display += '-'*numSpaces

        display = display + '\n'

        # print the weights
        for i,row in enumerate(self.localMap):
            for j,cell in enumerate(row):
                # print row heading
                if(j == 0):
                    display += printSpacedCharacter(i,numSpaces) + '|'
                if(j == self.size and i == self.size):
                    display += bcolors.OKBLUE + printSpacedCharacter('R',numSpaces) + bcolors.ENDC
                else:
                    if(cell == 1):
                        display += bcolors.FAIL + printSpacedCharacter(cell,numSpaces) + bcolors.ENDC
                    elif((i,j) in self.localPathList):
                        display += bcolors.OKGREEN + printSpacedCharacter(cell,numSpaces) + bcolors.ENDC
                    else:
                        display += printSpacedCharacter(cell,numSpaces)
            display += '\n'

        return display
    
    def printSpacedCharacter(self, char, numSpaces):
        '''
        Takes in a number of spaces and chars and outputs
        the character with the correct spacing
        '''
        spacedString = str(char)

        spacedString += (numSpaces - len(spacedString))*' '
        
        return spacedString
