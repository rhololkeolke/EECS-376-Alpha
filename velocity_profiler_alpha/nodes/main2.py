#!/usr/bin/env python

# Standard ros commands to make a node
import roslib; roslib.load_manifest('velocity_profiler_alpha');
import rospy

# message data types
from geometry_msgs.msg._Twist import Twist as TwistMsg
from geometry_msgs.msg._Point import Point as PointMsg
from std_msgs.msg._Bool import Bool as BoolMsg
from msg_alpha.msg._PathSegment import PathSegment as PathSegmentMsg
from msg_alpha.msg._Obstacles import Obstacles as ObstaclesMsg
from msg_alpha.msg._SegStatus import SegStatus as SegStatusMsg
from msg_alpha.msg._PathList import PathList as PathListMsg
from geometry_msgs.msg._PoseStamped import PoseStamped as PoseStampedMsg
from geometry_msgs.msg._Quaternion import Quaternion as QuaternionMsg

from math import sqrt
from collections import deque

from state import State
from trajseg import TrajSeg

# set the rate the node runs at
RATE = 20.0

# setup a Rate instance to keep the node running at the specified RATE
naptime = None # this will be initialized first thing in main

# stores the value of the E-stop
stopped = False

# stores the value of the obstacles
obs = ObstaclesMsg()

# stores the pathSegments by seg number
pathSegments = dict()

# stores the last velocity and omega commands
lastVCmd = 0.0
lastWCmd = 0.0

# stores the current best estimate of position and orientation
position = PointMsg()
orientation = QuaternionMsg()

# stores the computed trajectory
vTrajectory = deque()
wTrajectory = deque()

def eStopCallback(motors_enabled):
    global stopped
    stopped = not motors_enabled.data

def obstaclesCallback(obstacles):
    '''
    Updates the value of the E-stop
    '''
    global obs
    obs.exists = obstacles.exists
    obs.distance = obstacles.distance
    obs.ping_angle = obstacles.ping_angle

def pathListCallback(pathlist):
    '''
    Looks at the latest received path segment list.
    If there are changes it adds the pathSegments to the segments dictionary
    and recomputes the trajectory with the new segments
    '''
    global pathSegments
    # look and see if there are any changes
    changes = False
    for seg in pathlist.segments:
        if(pathSegments.get(seg.seg_number) is None): # if None then we have never seen this path segment before
            pathSegments[seg.seg_number] = seg # add this path segment to the dictionary
            changes = True # mark that there were new path segments so that the trajectory is recomputed
    if(changes):
        recomputeTrajectory(pathlist.segments)

def velCmdCallback(velocity):
    '''
    Updates the last values of velocity and omega commanded by steering
    '''
    global lastVCmd
    global lastWCmd
    lastVCmd = vel.linear.x
    lastWCmd = vel.angular.z

def poseCallback(pose):
    '''
    Updates the robots best estimate on position and orientation
    '''
    global position
    global orientation
    position = pose.pose.position
    orientation = pose.pose.orientation

def recomputeTrajectory(segments):
    '''
    This function takes in a list of path segments and returns a list of trajectory segments.
    It uses the final velocity of the previous segment as the initial velocity for the current segment
    For the first segment it uses the last velocity and omega commands as initial values
    '''
    global vTrajectory
    global wTrajectory
    vTrajSegs = [] # temporary holding place for all of the computed trajectory segments
    wTrajSegs = []

    # initial conditions are what the robot is currently experiencing as this segment
    lastV = lastVCmd
    lastW = lastWCmd

    nextV = 0.0
    nextW = 0.0
    print "============="
    print "Recomputing!"
    print "============="
    print ""
    for i,seg in enumerate(segments):
        # attempt to get the max speeds of the next segment
        # if there are no more segments after this then assume
        # the robot should be stopped
        try:
            nextSeg = segments[i+1]
            nextV = nextSeg.max_speeds.linear.x
            nextW = nextSeg.max_speeds.angular.z
        except IndexError:
            nextV = 0.0
            nextW = 0.0

        if(seg.seg_type == PathSegmentMsg.LINE):
            print "Computing trajectory for LINE segment number %i" % seg.seg_number
            print "\tWith v_i = %f" % lastV
            print "\tAnd v_f = %f" % nextV
            (vTempSegs, wTempSegs, lastV) = computeLineTrajectory(seg,lastV,nextV)
        elif(seg.seg_type == PathSegmentMsg.ARC):
            print "Computing trajectory for ARC segment number %i" % seg.seg_number
            print "\tWith v_i = %f" % lastV
            print "\tAnd v_f = %f" % nextV
            print "\tAnd w_i = %f" % lastW
            print "\tAnd w_f = %f" % nextW
            (vTempSegs, wTempSegs, lastV,lastW) = computeArcTrajectory(seg,lastV,nextV,lastW,nextW)
        elif(seg.seg_type == PathSegmentMsg.SPIN_IN_PLACE):
            print "Computing trajectory for SPIN_IN_PLACE segment number %i" % seg.seg_number
            print "\tWith w_i = %f" % lastW
            print "\tWith w_f = %f" % nextW
            (vTempSegs, wTempSegs, lastW) = computeSpinTrajectory(seg,lastW,nextW)
        else:
            print "Segment number %i is of unknown type!" % seg.seg_number
            print "\tSkipping..."
            vTempSegs = []
            wTempSegs = []

        vTrajSegs.extend(vTempSegs)
        wTrajSegs.extend(wTempSegs)
        
    vTrajectory.clear()
    vTrajectory.extend(vTrajSegs)
    wTrajectory.clear()
    wTrajectory.extend(wTrajSegs)
    print "-------------"
    print "Trajectories"
    print "-------------"
    print ""
    print vTrajectory
    print wTrajectory
    
def computeLineTrajectory(seg,v_i,v_f):
    '''
    Given a path segment of type LINE and the initial and final velocities compute the trajectory segments
    '''
    # omega should be zero the entire segment
    vTrajSegs = []
    wTrajSegs = [TrajSeg(TrajSeg.CONST,1.0,0.0,0.0,seg.seg_number)]

    return (vTrajSegs, wTrajSegs, v_f)

def computeArcTrajectory(seg,v_i,v_f,w_i,w_f):
    '''
    Given a path segment of type ARC and the initial and final velocities and initial and final omegas compute the trajectory segments
    '''
    vTrajSegs = []
    wTrajSegs = []

    return (vTrajSegs, wTrajSegs, v_f, w_f)

def computeSpinTrajectory(seg,w_i,w_f):
    '''
    Given a path segment of type SPIN_IN_PLACe and the initial and final omegas compute the trajectory segments
    '''
    # velocity should be zero the entire segment
    vTrajSegs = [TrajSeg(TrajSeg.CONST,1.0,0.0,0.0,seg.seg_number)]
    wTrajSegs = []

    return (vTrajSegs, wTrajSegs, w_f)
        
def main():
    global naptime
    
    rospy.init_node('velocity_profiler_alpha')
    naptime = rospy.Rate(RATE) # this will be used globally by all functions that need to loop
    desVelPub = rospy.Publisher('des_vel',TwistMsg) # Steering reads this and adds steering corrections on top of the desired velocities
    segStatusPub = rospy.Publisher('seg_status', SegStatusMsg) # Lets the other nodes know what path segment the robot is currently executing
    rospy.Subscriber("motors_enabled", BoolMsg, eStopCallback) # Lets velocity profiler know the E-stop is enabled
    rospy.Subscriber("obstacles", ObstaclesMsg, obstaclesCallback) # Lets velocity profiler know where along the path there is an obstacle 
    rospy.Subscriber("cmd_vel", TwistMsg, velCmdCallback) # 
    rospy.Subscriber("path", PathListMsg, pathListCallback)
    rospy.Subscriber("map_pos", PoseStampedMsg, poseCallback)
    
    print "Entering main loop"
    
    while not rospy.is_shutdown():
        naptime.sleep()
        """ print "stopped:"
        print stopped
        print "pathSegments:"
        print pathSegments.keys()
        print "lastVCmd:"
        print lastVCmd
        print "lastOCmd:"
        print lastOCmd
        print "position:"
        print position
        print "orientation"
        print orientation"""

        

if __name__ == "__main__":
    main()
