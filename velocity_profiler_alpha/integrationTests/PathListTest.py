#!/usr/bin/env python

import roslib; roslib.load_manifest('velocity_profiler_alpha');
import rospy

from geometry_msgs.msg._Twist import Twist as TwistMsg
from geometry_msgs.msg._Point import Point as PointMsg
from msg_alpha.msg._PathSegment import PathSegment as PathSegmentMsg
from msg_alpha.msg._PathList import PathList as PathListMsg
from tf.transformations import euler_from_quaternion,quaternion_from_euler
from geometry_msgs.msg._Quaternion import Quaternion as QuaternionMsg

import random
from math import pi,cos,sin

RATE = 1.0
naptime = None

def getYaw(quat):
    try:
        return euler_from_quaternion([quat.x,quat.y,quat.z, quat.w])[2]
    except AttributeError:
        return euler_from_quaternion(quat)[2]
    
def createQuat(x,y,z):
    quatList = quaternion_from_euler(x,y,z)
    quat = QuaternionMsg()
    quat.x = quatList[0]
    quat.y = quatList[1]
    quat.z = quatList[2]
    quat.w = quatList[3]
    return quat

def randomLines():
    global naptime
    
    rospy.init_node('path_list_test')
    naptime = rospy.Rate(RATE)
    pathListPub = rospy.Publisher('path',PathListMsg)

    print "Entering main loop"

    count = 0
    path = list()
    pathList = PathListMsg()
    while not rospy.is_shutdown():
        count += 1
        if(count % 5 == 0): # on multiples of 5 add a new path segment
            print "Appending a new segment"
            newSeg = PathSegmentMsg()
            newSeg.seg_number = count
            newSeg.seg_length = random.uniform(0,10)
            newSeg.seg_type = 1 #(count % 3) + 1
            newSeg.max_speeds.linear.x = random.uniform(0,10)
            newSeg.max_speeds.angular.z = random.uniform(0,10)
            newSeg.accel_limit = random.uniform(0,10)
            newSeg.decel_limit = random.uniform(-10,0)
            path.append(newSeg)
#        if(count % 8 == 0): # on multiples of 8 remove a segment from the list
 #           print "Deleting a segment"
  #          if(len(path) != 0):
   #             del path[count % len(path)]
    #    if(count % 23 == 0): # on multiples of 23 remove all segments from the list
     #       print "Deleting the list"
      #      path = list()

        pathList.segments = path
        pathListPub.publish(pathList)
        naptime.sleep()

def shortLines():
    global naptime
    
    rospy.init_node('path_list_test')
    naptime = rospy.Rate(20.0)
    pathListPub = rospy.Publisher('path',PathListMsg)

    print "Entering main loop"

    count = 0
    path = list()
    pathList = PathListMsg()
    lengths = [.2,1,.3,.5,.6]
    max_speeds = [1,.1,.25,.5,3]
    min_speeds = [0,0,0,0,0]
    accel = [.25,.5,.1,.25,1]
    decel = [-.25,-.2,-.2,-.2,-1]
    startx = 8.42
    starty = 15.09
    startYaw = pi/180.0*-132.39
    while not rospy.is_shutdown():
        if(count < len(lengths)):
            newSeg = PathSegmentMsg()
            newSeg.seg_type = PathSegmentMsg.LINE
            newSeg.seg_number = count
            newSeg.seg_length = lengths[count]
            newSeg.ref_point.x = startx
            newSeg.ref_point.y = starty
            if(count > 0):
                newSeg.ref_point.x += lengths[count]*cos(startYaw)
                newSeg.ref_point.y += lengths[count]*sin(startYaw)
            newSeg.init_tan_angle = createQuat(0,0,startYaw)
            newSeg.max_speeds.linear.x = max_speeds[count]
            newSeg.min_speeds.linear.x = min_speeds[count]
            newSeg.accel_limit = accel[count]
            newSeg.decel_limit = decel[count]
            path.append(newSeg)
            pathList.segments = path
            print "Appending segment number %i" % count
        pathListPub.publish(pathList)    
        count += 1
        naptime.sleep()

if __name__ == "__main__":
    shortLines()
