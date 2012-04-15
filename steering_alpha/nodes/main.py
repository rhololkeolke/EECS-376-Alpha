#!/usr/bin/env python

import roslib; roslib.load_manifest('steering_alpha')
import rospy

from msg_alpha.msg._PathSegment import PathSegment as PathSegmentMsg
from msg_alpha.msg._PathList import PathList as PathListMsg
from msg_alpha.msg._SegStatus import SegStatus as SegStatusMsg
from geometry_msgs.msg._Quaternion import Quaternion as QuaternionMsg
from geometry_msgs.msg._PoseStamped import PoseStamped as PoseStampedMsg
from geometry_msgs.msg._Point import Point as PointMsg
from geometry_msgs.msg._Twist import Twist as TwistMsg

from tf.transformations import quaternion_from_euler, euler_from_quaternion

from math import cos,sin,tan,pi,sqrt

# set the rate the node runs at
RATE = 20.0

# this will store a Rate instance to keep the node running at the specified RATE
naptime = None # this will be initialized first thing in main

# keeps track of the desired velocity
desVel = TwistMsg()

# the current segment definition to steer to
currSeg = None

# the last segment completed
lastSegComplete = 0

# pose data
position = PointMsg()
orientation = QuaternionMsg()

def segStatusCallback(segStat):
    '''
    Updates what the last segment completed was.
    pathListCallback uses this information to determine
    which path segment in the path segment list to steer
    to
    '''
    global lastSegComplete
    lastSegComplete = segStat.lastSegComplete

def desVelCallback(vel):
    '''
    Updates the desired velocity command specified
    by the velocity profiler
    '''
    global desVel
    desVel = vel

def pathListCallback(pathList):
    '''
    Looks at the latest received path segment list.
    It sets the current segment as the one with the
    lowest value that is larger than lastSegCompleted
    '''
    global currSeg
    
    # keeps track of the current best guess for the segment that should be steered to
    potentialSeg = None

    if(len(pathList.segments) != 0):
        # need to set the min to the largest number in the current set, to guaranteed
        # that a minimum larger than lastSegComplete is picked, if it exists
        minSegNumber = pathList.segments[-1].seg_number
    else:
        # if there are no path segments specified then there is no current segment
        currSeg = None
        return

    for seg in pathList.segments:
        if(seg.seg_number > lastSegComplete): # it may not the smallest, but it is an unfinished segment
            if(seg.seg_number < minSegNumber): # new minimum
                minSegNumber = seg.seg_number
                potentialSeg = seg

    # set the current segment to the actual best segment
    currSeg = potentialSeg

def poseCallback(pose):
    '''
    Updates the robot's best estimate on position and orientation
    '''
    global position
    global orientation
    position = pose.pose.position
    orientation = pose.pose.orientation
    
def getYaw(quat):
    try:
        return euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])[2]
    except AttributeError:
        return euler_from_quaternion(quat)[2]

def main():
    '''
    The main function that is executed while the node is running
    '''
    global RATE, desVel, naptime
    
    rospy.init_node('steering_alpha_main')
    naptime = rospy.Rate(RATE)

    cmdPub = rospy.Publisher('cmd_vel',TwistMsg)
    
    rospy.Subscriber('des_vel', TwistMsg, desVelCallback)
    rospy.Subscriber('path', PathListMsg, pathListCallback)
    rospy.Subscriber('map_pos', PoseStampedMsg, poseCallback)
    rospy.Subscriber('seg_status', SegStatusMsg, segStatusCallback)

    print "Entering main loop"

    while not rospy.is_shutdown():
        print "-------"
        print "desVel:"
        print "-------"
        print desVel
        print ""
        print "--------"
        print "currSeg:"
        print "--------"
        print currSeg
        print ""
        print "-------------------"
        print "lastSegComplete: %i" % lastSegComplete
        print "-------------------"
        print ""
        print "---------"
        print "position:"
        print "---------"
        print position
        print ""
        print "----"
        print "psi:"
        print "----"
        print "%f" % (getYaw(orientation))
        print ""
        naptime.sleep()

if __name__ == "__main__":
    main()
