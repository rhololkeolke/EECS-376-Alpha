#!/usr/bin/env python

import roslib; roslib.load_manifest('velocity_profiler_alpha');
import rospy

from geometry_msgs.msg._Twist import Twist as TwistMsg
from geometry_msgs.msg._Point import Point as PointMsg
from msg_alpha.msg._PathSegment import PathSegment as PathSegmentMsg
from msg_alpha.msg._PathList import PathList as PathListMsg

import random

RATE = 1.0
naptime = None

def main():
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
            newSeg.seg_length = count*10.0
            newSeg.seg_type = (count % 3) + 1
            newSeg.max_speeds.linear.x = random.uniform(0,10)
            newSeg.max_speeds.angular.z = random.uniform(0,10)
            path.append(newSeg)
        if(count % 8 == 0): # on multiples of 8 remove a segment from the list
            print "Deleting a segment"
            if(len(path) != 0):
                del path[count % len(path)]
        if(count % 23 == 0): # on multiples of 23 remove all segments from the list
            print "Deleting the list"
            path = list()

        pathList.segments = path
        pathListPub.publish(pathList)
        naptime.sleep()

if __name__ == "__main__":
    main()
