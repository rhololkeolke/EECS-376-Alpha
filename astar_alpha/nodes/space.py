class Space():
    def __init__(self,point,goal,parent=None):
        '''
        Constructor for space class. This class will be used in astar method.
        point is a tuple of the form (x,y) where x and y are the coordinates of the space.
        goal is a tuple of the form (goalx, goaly) where goalx and goaly are the coordinates of the goal space
        parent is an instance of space 
        '''
        from math import sqrt

        self.point = point
        self.parent = parent
        
        self.h = sqrt(pow(point[0]-goal[0],2) + pow(point[1]-goal[1],2))
        
        if parent is not None:
            self.g = parent.g + 1
        else:
            self.g = 0

    def f(self):
        return self.g + self.h

    def __lt__(self, other):
        return self.f() < other.f()
