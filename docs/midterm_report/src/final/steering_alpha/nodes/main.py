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
        #print "No Segments specified in pathList"
        currSeg = None
        return

    for seg in pathList.segments:
        if(seg.seg_number > lastSegComplete): # it may not the smallest, but it is an unfinished segment
            if(seg.seg_number <= minSegNumber): # new minimum
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

def getStartAndEndPoints():
    if(currSeg.seg_type == PathSegmentMsg.LINE):
        p_s = PointMsg()
        p_f = PointMsg()

        p_s.x = currSeg.ref_point.x
        p_s.y = currSeg.ref_point.y

        p_f.x = currSeg.ref_point.x + currSeg.seg_length*cos(getYaw(currSeg.init_tan_angle))
        p_f.y = currSeg.ref_point.y + currSeg.seg_length*sin(getYaw(currSeg.init_tan_angle))
        return (p_s,p_f)
    elif(currSeg.seg_type == PathSegmentMsg.ARC):
        return (0.0,0.0) # TODO
    elif(currSeg.seg_type == PathSegmentMsg.SPIN_IN_PLACE):
        return (currSeg.ref_point,currSeg.ref_point) # spins should never move in x,y plane
    else:
        return (0.0,0.0) # not sure if this is the best default answer

def getStartAndEndYaw():
    if(currSeg.seg_type == PathSegmentMsg.LINE):
        temp = getYaw(currSeg.init_tan_angle)
        return (temp,temp) # yaw of straight line shouldn't change
    elif(currSeg.seg_type == PathSegmentMsg.ARC):
        return (0.0,0.0)
    elif(currSeg.seg_type == PathSegmentMsg.SPIN_IN_PLACE):
        yaw_s = getYaw(currSeg.init_tan_angle)
        yaw_f = (yaw_s + currSeg.seg_length) % (2*pi)
        return (yaw_s,yaw_f)
    else:
        return (0.0,0.0) # not sure if this is the best default answer

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

    print "Steering entering main loop"
    
    Kd = 0.5
    Ktheta = 1.0

    while not rospy.is_shutdown():
        if(currSeg is not None and currSeg.seg_type == PathSegmentMsg.LINE):
            (p_s,p_f) = getStartAndEndPoints()
            desPsi = getYaw(currSeg.init_tan_angle)

            currPsi = getYaw(orientation)
            tx = cos(desPsi)
            ty = sin(desPsi)
            nx = -ty
            ny = tx

            dTheta = (desPsi - currPsi) % (2*pi)
            if(dTheta > pi):
                dTheta  = dTheta-2*pi
            
            # compute offset error
            currX = position.x
            currY = position.y

            # vector from start point to current robot point
            xrs = currX-p_s.x
            yrs = currY-p_s.y

            # dot this vectory with path normal vector to get the offset (works for line segments)
            offset = xrs*nx+yrs*ny

            cmd_vel = TwistMsg()
            cmd_vel.angular.z = -Kd*offset + Ktheta*dTheta
            cmd_vel.linear.x = desVel.linear.x

            cmdPub.publish(cmd_vel)
            naptime.sleep()
            continue
        elif(currSeg is not None):
            cmdPub.publish(desVel)
            naptime.sleep()
            continue
            #print "--------"
            #print "cmd_vel:"
            #print "--------"
            #print cmd_vel
            #print ""
        #print "-------"
        #print "desVel:"
        #print "-------"
        #print desVel
        #print ""
        #print "--------"
        #print "currSeg:"
        #print "--------"
        #print currSeg
        #print ""
        #print "-------------------"
        #print "lastSegComplete: %i" % lastSegComplete
        #print "-------------------"
        #print ""
        #print "---------"
        #print "position:"
        #print "---------"
        #print position
        #print ""
        #print "----"
        #print "psi:"
        #print "----"
        #print "%f" % (getYaw(orientation))
        #print ""
        cmdPub.publish(TwistMsg())
        naptime.sleep()

if __name__ == "__main__":
    main()
