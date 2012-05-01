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

    def updateClosedList(self,closedList,recompute=True):
        '''
        Will change the values in the closed list
        If any of the points in the closed list are on a path point
        Then the path will be recomputed, unless recompute is set
        to False.

        This method will return True if a new path was computed
        and false otherwise
        '''

        (conflict, newGrid) = self.populateGrid(closedList)

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
        from math import floor
        newGrid = self.createGrid()

        xStep = floor(abs(self.c1[0] - self.c2[0])/self.numCells)
        yStep = floor(abs(self.c1[1] - self.c2[1])/self.numCells)

        conflict = False
        # fill in the squares in the grid that are included in the current path
        for point in closedList:
            if(self.__pathDict.get(point,None)):
                conflict = True
            xIndex = int(floor(point[0]/xStep))
            yIndex = int(floor(point[1]/yStep))
            newGrid[xIndex][yIndex] = -1

        return (conflict, newGrid)
