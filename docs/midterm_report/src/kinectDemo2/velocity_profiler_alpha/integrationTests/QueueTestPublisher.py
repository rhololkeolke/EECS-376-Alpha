#!/usr/bin/env python
'''
Created on Apr 1, 2012

@author: Devin Schwab
'''

import roslib; roslib.load_manifest('velocity_profiler_alpha');
import rospy

from msg_alpha.msg._PathSegment import PathSegment as PathSegmentMsg

def main():
    rospy.init_node('QueueTestPublisher')
    pathSegPub = rospy.Publisher('path_seg',PathSegmentMsg)
    
    pathSeg = PathSegmentMsg()
    
    naptime = rospy.Rate(1)
    
    rospy.loginfo("About to start publishing")
    
    pathSegPub.publish(pathSeg)
    naptime.sleep()
    
    count = 1
    while not rospy.is_shutdown():
        rospy.loginfo("Publishing segment number %i" % (count))
        pathSeg.seg_number = count
        pathSegPub.publish(pathSeg)
        count += 1
        naptime.sleep()
        
    rospy.loginfo("Shutting down")

if __name__ == "__main__":
    main()