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

# keeps track of the percent complete of the current path segment
currSeg = None

# Keeps track of the last segment number completed
lastSegNumber = 0

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
    lastVCmd = velocity.linear.x
    lastWCmd = velocity.angular.z

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

    # Compute if acceleration segment is needed
    # Essentially finding the intersection of the line passing through the point (0,v_i)
    # with the maximum velocity. if v_i >= maximum velocity then
    # sAccel <= 0
    # Otherwise sAccel > 0
    sAccel = (pow(seg.max_speeds.linear.x,2) - pow(v_i,2))/(2*seg.accel_limit*seg.seg_length)
    
    # Compute Deceleration segment
    # Essentially finding the intersection of the line passing through the point (1,v_f)
    # with the maximum velocity.  If v_f >= maximum velocity then
    # sDecel >= 1
    # Otherwise sDecel < 1
    if(v_f < seg.min_speeds.linear.x):
        sDecel = 1-abs((pow(seg.max_speeds.linear.x,2)-pow(seg.min_speeds.linear.x,2))/(2*.8*seg.decel_limit*seg.seg_length))
    else:
        sDecel = 1-abs((pow(seg.max_speeds.linear.x,2)-pow(v_f,2))/(2*.8*seg.decel_limit*seg.seg_length))

    
    # Determine where accel and decel lines intersect.
    # if intersect at x < 0 then should only be decelerating and potentially const
    # if intersect at x > 0 then should only be accelerating and potentially const
    # if intersect in the middle then potentially should be doing all three
    xIntersect = (v_f-v_i-seg.decel_limit)/((seg.accel_limit-seg.decel_limit)*seg.seg_length)
    if(xIntersect < 0.0): # No acceleration
        if(sDecel >= 1): # should be travelling at a const velocity the whole segment
            temp = TrajSeg(TrajSeg.CONST,1.0,seg.max_speeds.linear.x,seg.max_speeds.linear.x,seg.seg_number)
            vTrajSegs.append(temp)
        if(sDecel < 0.0): # if this is less than 0 then decelerate the whole trip
            temp = TrajSeg(TrajSeg.DECEL,1.0,seg.max_speeds.linear.x,v_f,seg.seg_number)
            vTrajSegs.append(temp)
        else: # there is some constant velocity during this segment
            temp = TrajSeg(TrajSeg.CONST,sDecel,seg.max_speeds.linear.x,seg.max_speeds.linear.x,seg.seg_number)
            vTrajSegs.append(temp)
            temp = TrajSeg(TrajSeg.DECEL,1.0,seg.max_speeds.linear.x,v_f,seg.seg_number)
            vTrajSegs.append(temp)
    elif(xIntersect > 1.0): # No deceleration
        if(sAccel < 0.0): # actually have to start by decelerating
            sAccel = (pow(seg.max_speeds.linear.x,2)-pow(v_i,2))/(2*seg.decel_limit*seg.seg_length)
            if(sAccel >= 1.0): # There is no constant velocity
                temp = TrajSeg(TrajSeg.DECEL,1.0,v_i,seg.max_speeds.linear.x,seg.seg_number)
                vTrajSegs.append(temp)
            else: # there is a section of constant velocity
                temp = TrajSeg(TrajSeg.DECEL,sAccel,v_i,seg.max_speeds.linear.x,seg.seg_number)
                vTrajSegs.append(temp)
                temp = TrajSeg(TrajSeg.CONST,1.0,seg.max_speeds.linear.x,seg.max_speeds.linear.x,seg.seg_number)
                vTrajSegs.append(temp)
        elif(sAccel >= 1.0): # always accelerating
            temp = TrajSeg(TrajSeg.ACCEL,1.0,v_i,seg.max_speeds.linear.x,seg.seg_number)
            vTrajSegs.append(temp)
        else: # some constant velocity
            temp = TrajSeg(TrajSeg.ACCEL,sAccel,v_i,seg.max_speeds.linear.x,seg.seg_number)
            vTrajSegs.append(temp)
            temp = TrajSeg(TrajSeg.CONST,1.0,seg.max_speeds.linear.x,seg.max_speeds.linear.x,seg.seg_number)
            vTrajSegs.append(temp)
    else: # both acceleration and deceleration
        sLeft = 1.0 # whatever is left is the amount of time spent in constant velocity
        if(sAccel < 0.0): # should actually start with a deceleration
            sAccel = (pow(seg.max_speeds.linear.x,2)-pow(v_i,2))/(2*seg.decel_limit*seg.seg_length)
            if(sAccel > 1.0): # decelerating the entire time
                sAccel = 1.0
            temp = TrajSeg(TrajSeg.DECEL,sAccel,v_i,seg.max_speeds.linear.x,seg.seg_number)
            vTrajSegs.append(temp)
        elif(sAccel > 1.0): # will accelerate the whole time
            temp = TrajSeg(TrajSeg.ACCEL,1.0,v_i,seg.max_speeds.linear.x,seg.seg_number)
            vTrajSegs.append(temp)
        else:
            temp = TrajSeg(TrajSeg.ACCEL,sAccel,v_i,seg.max_speeds.linear.x,seg.seg_number)
            vTrajSegs.append(temp)

        sLeft -= sAccel
        decelSeg = None # used to temporarily store the deceleration segment if needed, segments have to be added in order
        # see if there is any s left
        if(sLeft > 0.0):
            if(sDecel < 1.0):
                decelSeg = TrajSeg(TrajSeg.DECEL,1.0,seg.max_speeds.linear.x,max(v_f,seg.min_speeds.linear.x),seg.seg_number)
                sLeft -= 1-sDecel
        
        if(sLeft > 0.0): # there is anything left in s then that is how long to do constant velocity for
            sAccel = (pow(seg.max_speeds.linear.x,2) - pow(v_i,2))/(2*seg.accel_limit*seg.seg_length)
            temp = TrajSeg(TrajSeg.CONST,sAccel+sLeft,seg.max_speeds.linear.x,seg.max_speeds.linear.x,seg.seg_number)
            vTrajSegs.append(temp)

        if(decelSeg is not None): # if there was a decel segment defined then add it to the vTrajSeg list
            vTrajSegs.append(decelSeg)
                
    return (vTrajSegs, wTrajSegs, max(v_f,seg.min_speeds.linear.x))

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
        
def getDesiredVelocity(vTrajSeg,wTrajSeg):
    '''
    Given a velocity trajectory segment and an omega trajectory segment this function will
    Compute the scheduled velocity and omega for the robot's current position along the path
    '''
    global vTrajectory
    global wTrajectory
    global pathSegments
    global currSeg


    print "segDistDone: %f" % (currSeg.segDistDone)
    if(vTrajSeg.segType == TrajSeg.ACCEL):
        #print "Using velocity acceleration segment"
        vCmd = getDesiredVelAccel(vTrajSeg, currSeg.segDistDone)
    elif(vTrajSeg.segType == TrajSeg.CONST):
        #print "Using constant velocity segment"
        vCmd = getDesiredVelConst(vTrajSeg, currSeg.segDistDone)
    elif(vTrajSeg.segType == TrajSeg.DECEL):
        #print "Using velocity deceleration segment"
        vCmd = getDesiredVelDecel(vTrajSeg, currSeg.segDistDone)
        print "vCmd: %f" % vCmd
   
    if(wTrajSeg.segType == TrajSeg.ACCEL):
        #print "Using omega acceleration segment"
        wCmd = getDesiredVelAccel(wTrajSeg, currSeg.segDistDone,1)
    elif(wTrajSeg.segType == TrajSeg.CONST):
        #print "Using constant omega segment"
        wCmd = getDesiredVelConst(wTrajSeg, currSeg.segDistDone,1)
    elif(wTrajSeg.segType == TrajSeg.DECEL):
        #print "Using omega deceleration segment"
        wCmd = getDesiredVelDecel(wTrajSeg, currSeg.segDistDone,1)
    
    vel_cmd = TwistMsg()
    vel_cmd.linear.x = vCmd
    vel_cmd.angular.z = wCmd

    return vel_cmd
        
def getDesiredVelAccel(seg, segDistDone, cmdType=0):
    pathSeg = pathSegments.get(seg.segNumber)
    a_max = pathSeg.accel_limit
    d_max = pathSeg.decel_limit
    if(cmdType == 1):
        lastCmd = lastWCmd
    else:
        lastCmd = lastVCmd

    if(segDistDone < 0.0): # this is to prevent the robot from sticking in place with negative path offset
        if(abs(lastCmd) < abs(seg.v_f)):
            vScheduled = lastCmd + a_max*1/RATE
        else:
            vScheduled = lastCmd
    else:
        if(a_max < 0.0):
            vScheduled = -1*sqrt(pow(seg.v_i,2) + 2*pathSeg.seg_length*segDistDone*abs(a_max))
        else:
            vScheduled = sqrt(pow(seg.v_i,2) + 2*pathSeg.seg_length*segDistDone*a_max)
        if(abs(vScheduled) < abs(a_max)*1/RATE):
            vScheduled = a_max*1/RATE

    if(abs(lastCmd) < abs(vScheduled)):
        vTest = lastCmd + a_max*1/RATE
        if(abs(vTest) < abs(vScheduled)):
            vCmd = vTest
        else:
            vCmd = vScheduled
    elif(abs(lastCmd) > abs(vScheduled)):
        vTest = lastCmd + (1.2*d_max*1/RATE)
        if(abs(vTest) > abs(vScheduled)):
            vCmd = vTest
        else:
            vCmd = vScheduled
    else:
        vCmd = vScheduled
    
    return vCmd

def getDesiredVelConst(seg, segDistDone, cmdType=0):
    pathSeg = pathSegments.get(seg.segNumber)
    a_max = pathSeg.accel_limit
    d_max = pathSeg.decel_limit
    vScheduled = seg.v_i
    # to enable the use of this method for both omega and velocity
    # simply set lastCmd to whichever variable is appropriate
    if(cmdType == 1):
        lastCmd = lastWCmd
    else:
        lastCmd = lastVCmd

    if(abs(lastCmd) < abs(vScheduled)):
        vTest = lastCmd + a_max*1/RATE
        if(abs(vTest) < abs(vScheduled)):
            vCmd = vTest
        else:
            vCmd = vScheduled
    elif(abs(lastCmd) > abs(vScheduled)):
        vTest = lastCmd + (1.2*d_max*1/RATE)
        if(abs(vTest) > abs(vScheduled)):
            vCmd = vTest
        else:
            vCmd = vScheduled
    else:
        vCmd = vScheduled
    return vCmd

def getDesiredVelDecel(seg, segDistDone, cmdType=0):
    pathSeg = pathSegments.get(seg.segNumber)
    a_max = pathSeg.accel_limit
    d_max = pathSeg.decel_limit
    if(cmdType == 1):
        lastCmd = lastWCmd
    else:
        lastCmd = lastVCmd

    if(segDistDone > 1.0): # this to prevent negative numbers in the sqrt
        vScheduled = seg.v_f
    elif(segDistDone < 0.0): # this is to prevent the robot from getting stuck before a segment completes
        vScheduled = seg.v_i
    else:
        if(d_max < 0.0):
            vScheduled = sqrt(pow(seg.v_f,2)+2*(1-segDistDone)*pathSeg.seg_length*abs(d_max))
        else:
            vScheduled = -1*sqrt(pow(seg.v_f,2)+2*(1-segDistDone)*pathSeg.seg_length*abs(d_max))

    if(abs(lastCmd) < abs(vScheduled)):
        vTest = lastCmd + a_max*1/RATE
        if(abs(vTest) < abs(vScheduled)):
            vCmd = vTest
        else:
            vCmd = vScheduled
    elif(abs(lastCmd) > abs(vScheduled)):
        vTest = lastCmd + (1.2*d_max*1/RATE)
        if(abs(vTest) > abs(vScheduled)):
            vCmd = vTest
        else:
            vCmd = vScheduled
    else:
        vCmd = vScheduled
    return vCmd

def update():
    '''
    This function is responsible for updating all the state variables each iteration of the node's main loop
    '''
    global currSeg
    global pathSegments
    global vTrajectory
    global wTrajectory
    global lastSegNumber
    
    oldVTraj = None
    oldWTraj = None

    if(len(vTrajectory) == 0 or len(wTrajectory) == 0):
        # Clear the deques because either they should both have elements or neither should
        vTrajectory.clear()
        wTrajectory.clear()
        pathSegments.clear() # clear out the path segments because they are now useless
        currSeg.newPathSegment()
        return

    # if it made it to here then there is at least one segment in vTrajectory and wTrajectory
    if(currSeg.pathSeg is None):
        currSeg.newPathSegment(pathSegments.get(vTrajectory[0].segNumber),position,State.getYaw(orientation))
        last_vel = TwistMsg()
        last_vel.linear.x = lastVCmd
        last_vel.angular.z = lastWCmd
        currSeg.updateState(last_vel,position,State.getYaw(orientation))

    
    if(currSeg.segDistDone >= vTrajectory[0].endS): # this segment is done
        oldVTraj = vTrajectory.popleft() # temporary storage, this will eventually be thrown away
        if(len(vTrajectory) == 0):
            lastSegNumber = oldVTraj.segNumber
            wTrajectory.clear()
            pathSegments.clear()
            currSeg.newPathSegment()
            return
        if(oldVTraj.segNumber != vTrajectory[0].segNumber):
            lastSegNumber = oldVTraj.segNumber
            wTrajectory.popleft() # could potentially be out of sync. This method does not account for that
            currSeg.newPathSegment(pathSegments.get(vTrajectory[0].segNumber),position,State.getYaw(orientation))
            pathSegments.pop(oldVTraj.segNumber) # remove no longer needed pathSegments

    if(currSeg.segDistDone >= wTrajectory[0].endS): # this segment is done
        oldWTraj = wTrajectory.popleft()
        if(len(wTrajectory) == 0):
            lastSegNumber = oldWTraj.segNumber
            vTrajectory.clear()
            pathSegments.clear()
            currSeg.newPathSegment()
            return
        if(oldWTraj.segNumber != wTrajectory[0].segNumber):
            lastSegNumber = oldWTraj.segNumber
            vTrajectory.popleft()
            currSeg.newPathSegment(pathSegments.get(wTrajectory[0].segNumber),position,State.getYaw(orientation))
            pathSegments.pop(oldWTraj.segNumber)
        
def publishSegStatus(segStatusPub,abort=False):
    segStat = SegStatusMsg()
    segStat.lastSegComplete = lastSegNumber
    segStat.abort = abort
    if(currSeg is not None):
        if(currSeg.pathSeg is not None):
            segStat.seg_number = currSeg.pathSeg.seg_number
        else:
            segStat.seg_number = 0
        segStat.progress_made = currSeg.segDistDone
    else:
        segStat.seg_number = 0
        segStat.progress_made = 0.0
        
    segStatusPub.publish(segStat)
    

def main():
    global naptime
    global currSeg

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
    
    currSeg = State(dt=1/RATE)
    while not rospy.is_shutdown():
        # check where the robot is
        last_vel = TwistMsg()
        last_vel.linear.x = lastVCmd
        last_vel.angular.z = lastWCmd
        if(currSeg.pathSeg is not None):
            currSeg.updateState(last_vel,position,State.getYaw(orientation))
        # check if there are segments to execute
        if(len(vTrajectory) != 0 or len(wTrajectory) != 0): # Note: Either both or neither should have 0 elements
            des_vel = getDesiredVelocity(vTrajectory[0],wTrajectory[0])
        else:
            des_vel = TwistMsg() # initialized to 0's by default
        desVelPub.publish(des_vel) # publish either the scheduled commands or 0's
        update() # remove completed segments and change currSeg's path segment when necessary
        publishSegStatus(segStatusPub)
        naptime.sleep()            

if __name__ == "__main__":
    main()
