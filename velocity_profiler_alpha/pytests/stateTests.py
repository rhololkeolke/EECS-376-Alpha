'''
Created on Mar 24, 2012

@author: Devin Schwab
'''
import unittest
from state import State
from msg_alpha.msg._PathSegment import PathSegment as PathSegmentMsg
from geometry_msgs.msg._Point import Point as PointMsg
from geometry_msgs.msg._Twist import Twist as TwistMsg

class Test(unittest.TestCase):

    def test_init_NoArgs(self):
        testState = State()
        self.assertEqual(testState.pathSeg, None, str(testState.pathSeg) + " is not None")
        self.assertEqual(testState.pathPoint, None, str(testState.pathPoint) + " is not None")
        self.assertEqual(testState.point, None, str(testState.point) + " is not None")
        self.assertEqual(testState.segDistDone, 0.0, str(testState.segDistDone) + " is not 0.0")
        self.assertEqual(testState.dt, 1/20.0, str(testState.dt) + " is not " + str(1/20.0))

    def test_init_PathSeg(self):
        pathSeg = PathSegmentMsg()
        testState = State(pathSeg)
        self.assertEqual(testState.pathSeg, pathSeg, str(testState.pathSeg) + " is not equal to " + str(pathSeg))
        self.assertEqual(testState.pathPoint, pathSeg.ref_point, str(testState.pathPoint) + " is not equal to " + str(pathSeg.ref_point))
        self.assertEqual(testState.point, pathSeg.ref_point, str(testState.point) + " is not equal to " + str(pathSeg.ref_point))
        self.assertEqual(testState.segDistDone, 0.0, str(testState.segDistDone) + " is not 0.0")
        self.assertEqual(testState.dt, 1/20.0, str(testState.dt) + " is not " + str(1/20.0))
    
    def test_init_PathSegAndPoint(self):
        pathSeg = PathSegmentMsg()
        point = PointMsg()
        testState = State(pathSeg, point)
        self.assertEqual(testState.pathSeg, pathSeg, str(testState.pathSeg) + " is not equal to " + str(pathSeg))
        self.assertEqual(testState.pathPoint, pathSeg.ref_point, str(testState.pathPoint) + " is not equal to " + str(pathSeg.ref_point))
        self.assertEqual(testState.point, point, str(testState.point) + " is not equal to " + str(point))
        self.assertEqual(testState.segDistDone, 0.0, str(testState.segDistDone) + " is not 0.0")
        self.assertEqual(testState.dt, 1/20.0, str(testState.dt) + " is not " + str(1/20.0))
        
    def test_init_dt(self):
        testState = State(dt=1/100.0)
        self.assertEqual(testState.pathSeg, None, str(testState.pathSeg) + " is not None")
        self.assertEqual(testState.pathPoint, None, str(testState.pathPoint) + " is not None")
        self.assertEqual(testState.point, None, str(testState.point) + " is not None")
        self.assertEqual(testState.segDistDone, 0.0, str(testState.segDistDone) + " is not 0.0")
        self.assertEqual(testState.dt, 1/100.0, str(testState.dt) + " is not " + str(1/100.0))   
    
    def test_newSegment_None(self):
        point = PointMsg() # initial point
        testState = State(point=point)
        testState.newPathSegment() # putting in a new segment shouldn't overwrite the point by default
        self.assertEqual(testState.pathSeg, None, str(testState.pathSeg) + " is not None")
        self.assertEqual(testState.pathPoint, None, str(testState.pathPoint) + " is not None")
        self.assertEqual(testState.point, point, str(testState.pathPoint) + " is not " + str(point))
        self.assertEqual(testState.segDistDone, 0.0, str(testState.segDistDone) + " is not 0.0")
        self.assertEqual(testState.dt, 1/20.0, str(testState.dt) + " is not " + str(1/20.0))
        
    def test_newSegment_PathSeg(self):
        point = PointMsg()  # initial point
        pathSeg = PathSegmentMsg()
        testState = State(point=point)
        testState.newPathSegment(pathSeg)
        self.assertEqual(testState.pathSeg, pathSeg, str(testState.pathSeg) + " is not " + str(pathSeg))
        self.assertEqual(testState.pathPoint, pathSeg.ref_point, str(testState.pathPoint) + " is not " + str(pathSeg.ref_point))
        self.assertEqual(testState.point, point, str(testState.point) + " is not " + str(point))
        self.assertEqual(testState.segDistDone, 0.0, str(testState.segDistDone) + " is not 0.0")
        self.assertEqual(testState.dt, 1/20.0, str(testState.dt) + " is not " + str(1/20.0))
        
    def test_newSegment_PathSegWithNonZeroDistance(self):
        testState = State() 
        testState.segDistDone = 5.0
        pathSeg = PathSegmentMsg()
        testState.newPathSegment(pathSeg)
        
        self.assertEqual(testState.pathSeg, pathSeg)
        self.assertEqual(testState.pathPoint, pathSeg.ref_point)
        self.assertEqual(testState.point, pathSeg.ref_point)
        self.assertEqual(testState.segDistDone, 0.0)
        self.assertEqual(testState.dt, 1/20.0)   

if __name__ == "__main__":
    #import sys;sys.argv = ['', 'Test.testName']
    unittest.main()