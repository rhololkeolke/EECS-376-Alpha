#!/usr/bin/env python
'''
Created on Apr 1, 2012

@author: Devin Schwab
'''

"""
This program will take in custom path definitions either in a csv format
or through the command line and send them to the robot.

This was created to be used during testing
"""

import roslib; roslib.load_manifest('velocity_profiler_alpha');
import rospy

from msg_alpha.msg._PathSegment import PathSegment as PathSegmentMsg
from geometry_msgs.msg._Point import Point as PointMsg
from geometry_msgs.msg._Twist import Twist as TwistMsg
from tf.transformations import quaternion_from_euler,euler_from_quaternion

import csv

def publishFromFile(fullPath):
    """
    Columns for file in order should be
    seg_type (1,2,3)
    seg_length (float)
    ref_point.x (float)
    ref_point.y (float)
    init_tan_angle (float)
    curvature (float)
    max_v (float)
    max_w (float)
    min_v (float)
    min_w (float)
    accel_limit (float)
    decel_limit (float)
    """
    
    with open(fullPath,'rb') as csvFile:
        dialect = csv.Sniffer().sniff(csvFile.read(1024)) # auto detect delimiters
        csvFile.seek(0)
        reader = csv.reader(csvFile, dialect) # open up a csv reader object with the csv file
        
        segs = []
        
        headers = next(reader)
        
        for i,row in enumerate(reader):
            pathSeg = PathSegmentMsg()
            pathSeg.seg_number = i+1

            try:
                seg_type = int(row[0])
            except ValueError:
                print "Problem reading %s" % row[0]
                print "\tMake sure the 1st column is a number"
                print "\tDefaulting seg_type to 1"
                seg_type = 1
            
            if(seg_type == 1):
                pathSeg.seg_type = PathSegmentMsg.LINE
            elif(seg_type == 2):
                pathSeg.seg_type = PathSegmentMsg.ARC
            elif(seg_type == 3):
                pathSeg.seg_type = PathSegmentMsg.SPIN_IN_PLACE
            else:
                print "Unknown segment type %s" % seg_type
                pathSeg.seg_type = 4 # this could be useful when testing incorrect input
            
            try:
                seg_length = float(row[1])
            except ValueError:
                print "Problem reading %s" % row[1]
                print "\tMake sure the 2nd column is a number"
                print "\tDefaulting to length 1.0"
                seg_length = 1.0
            pathSeg.seg_length = seg_length
            try: 
                x = float(row[2])
            except ValueError:
                print "Problem reading %s" % row[2]
                print "\tMake sure the 3rd column is a number"
                print "\tDefaulting x to 0.0"
                x = 0.0
            pathSeg.ref_point.x = x
            
            try:
                y = float(row[3])
            except ValueError:
                print "Problem reading %s" % row[3]
                print "\tMake sure the 5th column is a number"
                print "\tDefaulting y to 0.0"
                y = 0.0
            pathSeg.ref_point.y = y

            try:
                tan_angle = float(row[4])
            except ValueError:
                print "Problem reading %s" % row[4]
                print "\tMake sure the 6th column is a number"
                print "\tDefaulting tan_angle to 0.0"
                tan_angle = 0.0
            init_quat = quaternion_from_euler(0,0,tan_angle)
            pathSeg.init_tan_angle.w = init_quat[3]
            pathSeg.init_tan_angle.x = init_quat[0]
            pathSeg.init_tan_angle.y = init_quat[1]
            pathSeg.init_tan_angle.z = init_quat[2]
                    
            try:
                curvature = float(row[5])
            except ValueError:
                print "Problem reading %s" % row[5]
                print "\tMake sure the 7th column is a number"
                print "\tDefaulting curvature to 0.0"
                curvature = 0.0
            pathSeg.curvature = curvature
                    
                            
                        
                    
                

def publishFromCommandLine():
    pass

if __name__ == "__main__":
    import sys
    import os
    
    if(len(sys.argv) > 1):
        fileName = sys.argv[1]
        fullPath = os.path.join('.',fileName)
        
        publishFromFile(fullPath)
        
    else:
        publishFromCommandLine()