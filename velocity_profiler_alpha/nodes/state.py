#!/bin/usr/env python
'''
Created on Mar 23, 2012

@author: Devin Schwab
'''

from msg_alpha.msg._PathSegment import PathSegment as PathSegmentMsg
from geometry_msgs.msg._Twist import Twist as TwistMsg
from geometry_msgs.msg._Point import Point as PointMsg
from geometry_msgs.msg._Quaternion import Quaternion as QuaternionMsg
from geometry_msgs.msg._Vector3 import Vector3 as Vector3Msg

from tf.transformations import quaternion_from_euler,euler_from_quaternion # used to convert between the two representations
from math import cos,sin,tan,sqrt,pi

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
        
    def __init__(self, pathSeg=None, point = None, psi=None, dt=1/20.0):
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
            if(psi is not None): # a starting heading was given
                self.psi = psi
            else:
                self.psi = State.getYaw(pathSeg.init_tan_angle) # assume the robot is aligned with the path
            
            self.v = pathSeg.min_speeds.linear.x
            self.o = pathSeg.min_speeds.angular.z
        else:
            self.psi = psi # doesn't matter if psi is None or defined
            self.point = point # doesn't matter if point is None or defined
            self.pathPoint = None # there is no point along the path because there is no path
            self.v = 0.0
            self.o = 0.0
            
        self.segDistDone = 0.0 # by default the distance done must be zero
        self.dt = dt # set the timestep
        self.segDone = False
        
    
    def newPathSegment(self, pathSeg = None, point = None, psi = None):
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
        if(psi is not None):
            self.psi = psi
        else:
            if(pathSeg is not None):
                self.psi = State.getYaw(pathSeg.init_tan_angle)
        self.pathSeg = pathSeg # this will be None when
        if(pathSeg is not None): # as long as a path is specified
            self.pathPoint=pathSeg.ref_point # a pathPoint can be assumed
        self.segDistDone = 0.0 # new segment so completion is 0 
        
    
    def updateState(self, vel_cmd):
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
        
        # calculate the actual path vector using the code from the first assignment
        v_avg = (self.v + vel_cmd.linear.x)/2.0
        o_avg = (self.o + vel_cmd.angular.z)/2.0
        psi_avg = (self.psi + o_avg*self.dt)/2.0
            
        x = self.point.x + v_avg*self.dt*cos(psi_avg)
        y = self.point.y + v_avg*self.dt*sin(psi_avg)
        psi = self.psi + o_avg*self.dt
        
        # use segDistDone to calculate starting point on path
        if(self.pathSeg.seg_type == PathSegmentMsg.LINE):
            # calculate current position on path
            yaw = State.getYaw(self.pathSeg.init_tan_angle)
            x0 = self.pathSeg.ref_point.x + self.segDistDone*cos(yaw)
            y0 = self.pathSeg.ref_point.y + self.segDistDone*sin(yaw)

#            print "yaw: %f, x0: %f, y0: %f" % (yaw,x0,y0)

            # calculate maximum distance moved along path
            dDist = self.pathSeg.max_speeds.linear.x*self.dt
            
            # calculate ideal path position after next time step
            x1 = self.pathSeg.ref_point.x + (self.segDistDone+dDist)*cos(yaw)
            y1 = self.pathSeg.ref_point.y + (self.segDistDone+dDist)*sin(yaw)
            
            # calculate the ideal path vector
            idealVec = State.createVector([x0,y0,0.0], [x1,y1,0.0])
            
            # calculate the actual path vector
            actVec = State.createVector([self.point.x,self.point.y,self.point.z],[x,y,0.0])
            
            # create a unit vector in the same direction as the ideal path vector
            idealUnitVec = State.getUnitVector(idealVec)
            
            # calculate how far along the path the robot actually moved and add it to segDistDone
            self.segDistDone += State.dotProduct(actVec, idealUnitVec)

        elif(self.pathSeg.seg_type == PathSegmentMsg.ARC):
            rhoDes = self.pathSeg.curvature
            r = 1/abs(rhoDes) # turn radius is inverse of curvature
            yaw = State.getYaw(self.pathSeg.init_tan_angle)
            if(rhoDes >= 0):
                arcAngStart = yaw - pi/2.0
            else:
                arcAngStart = yaw + pi/2.0

            # calculate current position on path
            dAng = self.segDistDone*rhoDes
            arcAng0 = arcAngStart+dAng
            x0 = self.point.x + r*cos(arcAng0)
            y0 = self.point.y + r*sin(arcAng0)
            
            # calculate maximum distance moved along path
            arcAng1 = arcAng0 + self.pathSeg.max_speeds.angular.z*self.dt
            x1 = self.point.x + r*cos(arcAng1)
            y1 = self.point.y + r*sin(arcAng1)
            
            # calculate ideal path vector
            idealVec = State.createVector([x0,y0,0.0],[x1,y1,0.0])
            
            # calculate the actual path vector
            actVec = State.createVector([self.point.x,self.point.y,self.point.z], [x,y,0.0])
            
            # calculate the unit vector in ideal path vector direction
            idealUnitVec = State.getUnitVector(idealVec)
            
            # calculate how far along the path the robot actually moved and add it to segDistDone
            self.segDistDone += State.dotProduct(actVec, idealUnitVec)
            
        elif(self.pathSeg.seg_type == PathSegmentMsg.SPIN_IN_PLACE):
            yaw = State.getYaw(self.pathSeg.init_tan_angle)
            self.segDistDone = psi - yaw # distance done is the current heading minus the starting heading
        else:
            pass # should probably thrown an unknown segment type error
        
        # update the current velocity
        self.v = vel_cmd.linear.x
        self.o = vel_cmd.angular.z
        
        # update the current point and heading
        self.point.x = x
        self.point.y = y
        self.psi = psi

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
        self.v = 0.0
        self.o = 0.0
    
    @staticmethod
    def getYaw(quat):
        return euler_from_quaternion([quat.x,quat.y,quat.z, quat.w])[2]
    
    @staticmethod
    def createQuat(x,y,z):
        quatList = quaternion_from_euler(x,y,z)
        quat = QuaternionMsg()
        quat.x = quatList[0]
        quat.y = quatList[1]
        quat.z = quatList[2]
        quat.w = quatList[3]
        return quat
    
    @staticmethod
    def createVector(p0,p1):
        vector = Vector3Msg()
        vector.x = p1[0] - p0[0]
        vector.y = p1[1] - p0[1]
        vector.z = p1[2] - p0[2]
        return vector
    
    @staticmethod
    def getUnitVector(vector):
        e = Vector3Msg()
        magnitude = sqrt(pow(vector.x,2) + pow(vector.y,2) + pow(vector.z,2))
        try:
            e.x = vector.x/magnitude
            e.y = vector.y/magnitude
            e.z = vector.z/magnitude
        except ZeroDivisionError:
            e.x = 0.0
            e.y = 0.0
            e.z = 0.0
        return e
    
    @staticmethod
    def dotProduct(vec1, vec2):
        result = 0.0
        result += vec1.x*vec2.x
        result += vec1.y*vec2.y
        result += vec1.z*vec2.z
        return result
    
    
        
