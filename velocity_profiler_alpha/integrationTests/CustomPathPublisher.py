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
from tf.transformations import quaternion_from_euler

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
        rospy.init_node('CustomPathPublisher')
        pathSegPublisher = rospy.Publisher('path_seg',PathSegmentMsg) 
        
        dialect = csv.Sniffer().sniff(csvFile.read(1024)) # auto detect delimiters
        csvFile.seek(0)
        reader = csv.reader(csvFile, dialect) # open up a csv reader object with the csv file
        
        segs = []
        segs.append(PathSegmentMsg())
        
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
                print "\tMake sure the 4th column is a number"
                print "\tDefaulting y to 0.0"
                y = 0.0
            pathSeg.ref_point.y = y

            try:
                tan_angle = float(row[4])
            except ValueError:
                print "Problem reading %s" % row[4]
                print "\tMake sure the 5th column is a number"
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
                print "\tMake sure the 6th column is a number"
                print "\tDefaulting curvature to 0.0"
                curvature = 0.0
            pathSeg.curvature = curvature
            
            try:
                max_v = float(row[6])
            except ValueError:
                print "Problem reading %s" % row[6]
                print "\tMake sure the 7th column is a number"
                print "\tDefaulting max_v to 1.0"
                max_v = 1.0
            pathSeg.max_speeds.linear.x = max_v
            
            try:
                max_w = float(row[7])
            except ValueError:
                print "Problem reading %s" % row[7]
                print "\tMake sure the 8th column is a number"
                print "\tDefaulting max_w to 1.0"
                max_w = 1.0
            pathSeg.max_speeds.angular.z = max_w
            
            try:
                min_v = float(row[8])
            except ValueError:
                print "Problem reading %s" % row[8]
                print "\tMake sure the 9th column is a number"
                print "\tDefaulting min_v to 0.0"
                min_v = 0.0
            pathSeg.min_speeds.linear.x = min_v
            
            try:
                min_w = float(row[9])
            except ValueError:
                print "Problem reading %s" % row[9]
                print "\tMake sure the 10th column is a number"
                print "\tDefaulting min_w to 0.0"
                min_w = 0.0
            pathSeg.min_speeds.angular.z = min_w
            
            try:
                accel_limit = float(row[10])
            except ValueError:
                print "Problem reading %s" % row[10]
                print "\tMake sure the 11th column is a number"
                print "\tDefaulting accel_limit to 0.5"
                accel_limit = 0.5
            pathSeg.accel_limit = accel_limit
            
            try:
                decel_limit = float(row[11])
            except ValueError:
                print "Problem reading %s" % row[11]
                print "\tMake sure the 12th column is a number"
                print "\tDefaulting decel_limit to 0.5"
                decel_limit = 0.5
            pathSeg.decel_limit = decel_limit
            
            segs.append(pathSeg)
    
        print "About to publish"
        naptime = rospy.Rate(1)
        for pathSeg in segs:
            print "Publishing path segment %i" % (pathSeg.seg_number)
            print pathSeg
            pathSegPublisher.publish(pathSeg)
            naptime.sleep()
    
                

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