#!/usr/bin/env python
'''
Created on Apr 1, 2012

@author: Devin Schwab
'''

import roslib; roslib.load_manifest('velocity_profiler_alpha');
import rospy

from msg_alpha.msg._PathSegment import PathSegment as PathSegmentMsg
from Queue import Queue

pathSegs = Queue()

def pathSegCallback(pathSeg):
    global pathSegs
    rospy.loginfo("Putting segment number %i in queue" % (pathSeg.seg_number))
    pathSegs.put(pathSeg, True)
    
def main():
    global pathSegs
    rospy.init_node('QueueTestListener')
    rospy.Subscriber("path_seg", PathSegmentMsg, pathSegCallback) 
    
    naptime = rospy.Rate(.1)
    
    rospy.loginfo("About to start listening...")
    while not rospy.is_shutdown():
        if(pathSegs.qsize() > 0):
            pathSeg = pathSegs.get(True)
            rospy.loginfo("Path segment number %i removed from queue" % (pathSeg.seg_number))
        naptime.sleep()
    
    rospy.loginfo("Done listening...")
    rospy.loginfo("Printing rest of queue")
    while(pathSegs.qsize() > 0):
        pathSeg = pathSegs.get()
        rospy.loginfo("Path segment number %i removed from queue" % (pathSeg.seg_number))
        naptime.sleep()
        
    rospy.loginfo("Nothing left in queue, shutting down")
    
if __name__ == "__main__":
    main()
    
    