#!/bin/usr/env python
'''
Created on Mar 23, 2012

@author: Devin Schwab
'''

from msg_alpha.msg._PathSegment import PathSegment as PathSegmentMsg
from geometry_msgs.msg._Twist import Twist as TwistMsg
from geometry_msgs.msg._Point import Point as PointMsg

from tf.transformations import quaternion_from_euler,euler_from_quaternion # used to convert between the two representations


class State:
    """
    This class is responsible for keeping track of the robot's progress along the
    specified path segments.  It does so by integrating velocity commands with the
    assumption the robot is perfectly following the specified path.  It then uses the
    provided position information to calculate the actual path vector.  The projection
    of the actual path vector along the desired path vector is used to determine the
    current position on the path.
    
    Calculating the path in this way means that steering corrections and other perturbations
    shouldn't have a major effect on where velocity profiler thinks it is along a path.
    """
        
    def __init__(self, pathSeg=None, point = None, dt=1/20.0):
        """
        Responsible for initializing the State class
        
        Inputs
        ------
        pathSeg is of type PathSegmentMsg
        point is of type PointMsg
        dt is a floating point
        """
        self.pathSeg = pathSeg
        if(pathSeg is not None): # a path segment is already specified
            self.pathPoint = pathSeg.ref_point # the point the robot is projected on the path
            if(point is not None): # a starting point was given
                self.point = point # so set the point to that point
            else: # no starting point was given
                self.point = pathSeg.ref_point # set assume the robot is starting from the path segments beginning
        else:
            self.point = point # doesn't matter if point is None or defined
            self.pathPoint = None # there is no point along the path because there is no path
            
        self.segDistDone = 0.0 # by default the distance done must be zero
        self.dt = dt # set the timestep
        
    
    def newPathSegment(self, pathSeg = None, point = None):
        """
        Updates the internal path segment that State is integrating against
        
        Inputs
        ------
        pathSeg is expected to be of type PathSegment
        point is expected to be of type Point
        
        Returns
        -------
        Nothing
        """
        if(point is not None): # if a new point is specified
            self.point = point # then set the state with that point, otherwise leave the point alone
        else: # no point was originally specified
            if(pathSeg is not None): # the new path segment exists
                self.point = pathSeg.ref_point # so assume that the point is at the start of the path segment
        self.pathSeg = pathSeg # this will be None when
        if(pathSeg is not None): # as long as a path is specified
            self.pathPoint=pathSeg.ref_point # a pathPoint can be assumed
        self.segDistDone = 0.0 # new segment so completion is 0 
        
    
    def updateState(self, vel_cmd, point):
        """
        Integrates along the desired path vector and projects the actual path 
        vector onto the desired vector
        
        Inputs
        ------
        vel_cmd is expected to be of type Twist
        point is expected to be of type Point
        
        Returns
        -------
        True if everything went okay
        False if an error occurred
        """
        if(self.point is None or self.pathSeg is None):
            return # can't integrate if no initial value and/or no path is given
        
        
            
    
    def stop(self):
        """
        Called when Estop is activated.  Instantly sets the last velocities to 0
        
        Inputs
        ------
        Nothing
        
        Returns
        -------
        Nothing
        """
        pass
    
    
        