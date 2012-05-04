class BrushFire():
    def __init__(self, c1, c2):
        pass
    def updateGlobalGrid():
        pass
    def extractLocal():
        pass
    def transformGridToMap(self, point):
        pass
    def transformMapToGrid(self, point):
        pass
    def getNeighbors(self, point):
        '''
        given a point in grid space find all of viable neighbors
        return this as a list of tuples
        '''
        neighbors = list()

        # up
        x = point[0]
        y = point[1] + 1
        if(y < self.height and y >=0 and x < self.height and x >= 0):
            neighbors.append((x,y))

        # up right
        x = point[0] + 1
        y = point[1] + 1
        if(y < self.height and y >=0 and x < self.height and x >= 0):
            neighbors.append((x,y))

        # right
        x = point[0] + 1
        y = point[1]
        if(y < self.height and y >=0 and x < self.height and x >= 0):
            neighbors.append((x,y))

        # down right
        x = point[0] + 1
        y = point[1] - 1
        if(y < self.height and y >=0 and x <self.height and x >= 0):
            neighbors.append((x,y))

        # down
        x = point[0]
        y = point[1] - 1
        if(y < self.height and y >=0 and x < self.height and x >= 0):
            neighbors.append((x,y))

        # down left
        x = point[0] - 1
        y = point[1] - 1
        if(y < self.height and y >=0 and x < self.height and x >= 0):
            neighbors.append((x,y))

        # left
        x = point[0] - 1
        y = point[1]
        if(y < self.height and y >=0 and x < self.height and x >= 0):
            neighbors.append((x,y))

        # up left
        x = point[0] - 1
        y = point[1] + 1
        if(y < self.height and y >=0 and x < self.height and x >= 0):
            neighbors.append((x,y))

        return neighbors

    def brushfire(self, localMap):
        '''
        Given a square grid of obstacles runs brushfire and returns grid
        '''
        self.height = len(localMap)
        seenZero = True
        # if there are no zeros seen in a loop, we are done with brushfire
        while seenZero:
            seenZero = False
            for r,row in enumerate(localMap):
                for c,col in enumerate(localMap[r]):
                    if localMap[r][c] == 0:
                        seenZero = True
                        neighbors = self.getNeighbors((r,c))
                        for point in neighbors:
                            localMap[point[0]][point[1]]+=1
        return localMap

    def computePath(self, localMap, goal):
        '''
        take grid of points passed through brushfire and returns list of points
        to follow
        '''
        

    def updateGoal():
        pass
