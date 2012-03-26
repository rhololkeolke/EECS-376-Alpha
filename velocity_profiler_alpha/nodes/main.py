#!/usr/bin/evn python
'''
Created on Mar 22, 2012

@author: Devin Schwab
'''

# Standard ros commands to make a node
import roslib; roslib.load_manifest('velocity_profiler_alpha');
import rospy

# message data types
from geometry_msgs.msg._Twist import Twist as TwistMsg
from std_msgs.msg._Bool import Bool as BoolMsg
from msg_alpha.msg._PathSegment import PathSegment as PathSegmentMsg 
from msg_alpha.msg._Obstacles import Obstacles as ObstaclesMsg 
from msg_alpha.msg._SegStatus import SegStatus as SegStatusMsg

from math import abs,sqrt,cos,sin

from state import State

RATE = 20.0 # set the rate of refresh

stopped = False # stores the value of the E-stop

# stores the values of obstacles in the way
obs = False
obs_dist = 0.0

lastVCmd = 0.0
lastOCmd = 0.0

seg_number = 0

# true when currSeg and nextSeg have actual values we want to follow
currSegExists = False
nextSegExists = False
nextSeg = PathSegmentMsg()
currSeg = PathSegmentMsg()

def eStopCallback(eStop):
    global stopped
    stopped = not eStop.data

def obstaclesCallback(obsData):
    global obs
    global obs_dist
    
    obs = obsData.exists
    obs_dist = obsData.distance

def pathSegmentCallback(seg):
    global nextSeg
    global nextSegExists
    
    if(seg.seg_number != nextSeg.seg_number):
        nextSegExists = True
        
    nextSeg.seg_number = seg.seg_number
    nextSeg.seg_type = seg.seg_type
    nextSeg.curvature = seg.curvature
    nextSeg.seg_length = seg.seg_length
    nextSeg.ref_point = seg.ref_point
    nextSeg.init_tan_angle = seg.init_tan_angle
    nextSeg.max_speeds = seg.max_speeds
    nextSeg.accel_limit = seg.accel_limit
    nextSeg.decel_limit = seg.decel_limit

def velCmdCallback(vel):
    global lastVCmd
    global lastOCmd
    lastVCmd = vel.linear.x
    lastOCmd = vel.angular.z
    
def straight(desVelPub,segStatusPub,distance):
    global RATE
    global lastVCmd
    global lastOCmd
    global obs
    global obs_dist
    global stopped
    global seg_number
    global currSegExists
    global currSeg
    global nextSegExists
    global nextSeg
    
    vel_object = TwistMsg()
    
    naptime = rospy.Rate(RATE)
    
    dt = 1/RATE
    if(distance < 0.0):
        v_max = -0.5
        a_max = -0.25
        segLength = -1*distance
    else:
        v_max = 0.5
        a_max = 0.25
        segLength = distance
        
    segDistDone = 0.0
    
    tAccel = rospy.Duration(v_max/a_max)
    #tDecel = rospy.Duration(tAccel)
    distAccel = 0.5*abs(a_max)*pow(tAccel.to_sec(),2)
    distDecel = distAccel
    distConstV = segLength - distAccel - distDecel
    #tConstV = rospy.Duration(distConstV/abs(v_max))
    #tSegmentTot = tAccel + tDecel + tConstV
    
    currState = State()
    
    v_cmd = lastVCmd
    omega_cmd = lastOCmd
    v_scheduled = 0.0
    
    if(a_max < 0.0):
        rospy.loginfo("Started a straight line segment for " + str(segLength) + " meters in the negative x direction")
    else:
        rospy.loginfo("Started a straight line segment for " + str(segLength) + " meters in the positive x direction")
    
    lastStopped = stopped
    lastObs = False
    startingPos = 0.0
    #obsDist = 0.0
    decelRate = 0.0
    
    while(segDistDone < segLength and not rospy.is_shutdown()):
        v_cmd = lastVCmd
        omega_cmd = lastOCmd
        
        if(stopped):
            lastStopped = stopped
            rospy.loginfo("STOPPED!")
            v_cmd = 0
            omega_cmd = 0
            currState.stop()
            
            status = SegStatusMsg()
            status.segComplete = False
            status.seg_number = seg_number
            status.progress_made = currState.segDistDone
            
            segStatusPub.publish(status)
            
            vel_object.linear.x = 0.0
            vel_object.angular.z = 0.0
            
            desVelPub.publish(vel_object)
            
            naptime.sleep()
            continue
        elif(lastStopped):
            rospy.loginfo("Sleeping for 2.0 seconds")
            lastStopped = False
            rospy.sleep(rospy.Duration(2.0))
            
        if(obs and not lastObs):
            rospy.loginfo("Initializing obstacles")
            startingPos = currState.segDistDone
            #obsDist = obs_dist
            lastObs = True
            decel_rate = pow(v_cmd,2)/(obs_dist-.1)
            rospy.loginfo("obs_dist: %f, decel_rate: %f, startingPos: %f" %(obs_dist,decel_rate,startingPos))
        
        if(lastObs and obs and abs(distance)-startingPos >= obs_dist):
            rospy.loginfo("Running obstacle code")
            
            if(v_cmd > .001):
                v_cmd = v_cmd - decelRate*dt
                vel_cmd = TwistMsg()
                vel_cmd.linear.x = v_cmd
                vel_cmd.angular.z = omega_cmd
                currState.updateState(vel_cmd)
                vel_object.linear.x = v_cmd
            else:
                vel_object.linear.x = 0.0
                currState.stop()
            
            status = SegStatusMsg()
            status.segComplete = False
            status.seg_number = seg_number
            status.progress_made = currState.segDistDone
            
            segStatusPub.publish(status)
            
            vel_object.angular.z = 0.0
            desVelPub.publish(vel_object)
            
            naptime.sleep()
            continue
        else:
            lastObs = False
            
        vel_cmd = TwistMsg()
        vel_cmd.linear.x = v_cmd
        vel_cmd.angular.z = omega_cmd
        currState.updateState(vel_cmd)
        
        segDistDone = currState.segDistDone
        v_cmd = currState.v
        if(segDistDone < distAccel):
            if(a_max < 0.0):
                v_scheduled = -1*sqrt(2*segDistDone*abs(a_max))
            else:
                v_scheduled = sqrt(2*segDistDone*a_max)
            if(abs(v_scheduled) < abs(a_max)*dt):
                v_scheduled - a_max*dt
        elif(segDistDone < distAccel+distConstV):
            v_scheduled = v_max
        else:
            if(a_max < 0.0):
                v_scheduled = -1*sqrt(2*(segLength-segDistDone)*abs(a_max))
            else:
                v_scheduled = sqrt(2*(segLength-segDistDone)*a_max)
                
        if(abs(v_cmd) < abs(v_scheduled)):
            v_test = v_cmd + a_max*dt
            if(abs(v_test) < abs(v_scheduled)):
                v_cmd = v_test
            else:
                v_cmd = v_scheduled
        elif(abs(v_cmd) > abs(v_scheduled)):
            v_test = v_cmd - (1.2*a_max*dt)
            if(abs(v_test) > abs(v_scheduled)):
                v_cmd = v_test
            else:
                v_cmd = v_scheduled

        status = SegStatusMsg()
        status.segComplete = False
        status.seg_number = seg_number
        status.progress_made = currState.segDistDone

        segStatusPub.publish(status)
        
        vel_object.linear.x = v_cmd
        vel_object.angular.z = 0.0
        desVelPub.publish(vel_object)
        
        naptime.sleep()
    
    currSegExists = False
    
    status = SegStatusMsg()
    status.segComplete = True
    status.seg_number = seg_number
    status.progress_made = currState.segDistDone
    
    segStatusPub.publish(status)
    
    vel_object.linear.x = 0.0
    vel_object.angular.z = 0.0
    desVelPub.publish(vel_object)


def turn(desVelPub, segStatusPub, angle):
    global RATE
    global lastVCmd
    global lastOCmd
    global obs
    global obs_dist
    global stopped
    global seg_number
    global currSegExists
    global currSeg
    global nextSegExists
    global nextSeg
    
    vel_object = TwistMsg()
    
    naptime = rospy.Rate(RATE)
    
    dt = 1/RATE
    if(angle < 0.0):
        o_max = -1.0
        a_max = -0.5
        segRads = -1*angle
    else:
        o_max = 1.0
        a_max = 0.5
        segRads = angle
        
    segRadsDone = 0.0
    
    tAccel = rospy.Duration(o_max/a_max)
    #tDecel = tAccel
    radAccel = 0.5*abs(a_max)*pow(tAccel.to_sec(),2)
    radDecel = radAccel
    radConstO = segRads - radAccel - radDecel
    #tConstV = rospy.Duration(radConstO/abs(o_max))
    #tSegmentTot = tAccel + tDecel + tConstV
    
    currState = State()
    
    v_cmd = 0.0
    o_cmd = 0.0
    o_scheduled = 0.0
    
    if(a_max < 0.0):
        rospy.loginfo("Started turning for %f radians, in the negative direction about the z axis" % (segRads))
    else:
        rospy.loginfo("Started turning for %f radians, in the positive direction about the z axis" %(segRads))

    
    lastStopped = stopped
    while(segRadsDone < segRads and not rospy.is_shutdown()):
        v_cmd = lastVCmd
        o_cmd = lastOCmd

        if(stopped):
            lastStopped = stopped
            rospy.loginfo("STOPPED!")
            currState.stop()
            
            v_cmd = 0.0
            o_cmd = 0.0
            
            vel_object.linear.x = 0.0
            vel_object.angular.z = 0.0
            
            status = SegStatusMsg()
            status.segComplete = False
            status.seg_number = seg_number
            status.progress_made = currState.segDistDone
            
            segStatusPub.publish(status)

            desVelPub.publish(vel_object)
            naptime.sleep()
            continue
        elif(lastStopped):
            rospy.loginfo("Sleeping for 2.0 seconds")
            lastStopped = False
            rospy.sleep(rospy.Duration(2.0))

        vel_cmd = TwistMsg()
        vel_cmd.linear.x = v_cmd
        vel_cmd.angular.z = o_cmd
        currState.updateState(vel_cmd)
        
        segRadsDone = currState.segDistDone()
        o_cmd = currState.o
        if(segRadsDone < radAccel):
            if(a_max < 0.0):
                o_scheduled = -1*sqrt(2*segRadsDone*abs(a_max))
            else:
                o_scheduled = sqrt(2*segRadsDone*a_max)
        
    
        if(abs(o_scheduled) < abs(a_max)*dt):
            o_scheduled = a_max*dt
        elif(segRadsDone < radAccel+radConstO):
            o_scheduled = o_max
        else:
            if(o_max < 0.0):
                o_scheduled = -1*sqrt(2*(segRads-segRadsDone)*abs(a_max))
            else:
                o_scheduled = sqrt(2*(segRads-segRadsDone)*a_max)
                
        if(abs(o_cmd) < abs(o_scheduled)):
            o_test = o_cmd + a_max*dt
            if(abs(o_test) < abs(o_scheduled)):
                o_cmd = o_test
            else:
                o_cmd = o_scheduled
        elif(abs(o_cmd) > abs(o_scheduled)):
            o_test = o_cmd - (1.2*a_max*dt)
            if(abs(o_test) > abs(o_scheduled)):
                o_cmd = o_test
            else:
                o_cmd = o_scheduled
        
        status = SegStatusMsg()
        status.segComplete = False
        status.seg_number = seg_number
        status.progress_made = currState.segDistDone
        
        segStatusPub.publish(status)
        
        vel_object.linear.x = 0.0
        vel_object.angular.z = o_cmd
        desVelPub.publish(vel_object)
        
        naptime.sleep()


    currSegExists = False
    
    status = SegStatusMsg()
    status.segComplete = True
    status.seg_number = seg_number
    status.progress_made = currState.segDistDone
    
    segStatusPub.publish(status)
    
    vel_object.linear.x = 0.0
    vel_object.angular.z = 0.0
    desVelPub.publish(vel_object)


def arc(desVelPub, segStatusPub, distance):
    pass

"""
This function is responsible for velocity profiling.  

In general velocity profiler takes in path segments from path publisher
and creates a trajectory based on the segments. The trajectory will smooth
the segments into each other based on the acceleration and velocity 
constraints of the current path segment and the future line segments. 
The velocity and acceleration constraints of the path segment are assumed 
to be slow enough to not cause wheel slip.

Velocity profiler is also in charge of keeping track of how far along the
trajectory the robot actually is.  It creates velocity commands based on where
along the trajectory the robot is. It publishes these commands to the steering node.

Velocity profiler reads in the Obstacles message and checks for obstacles.  If an
obstacle is detected velocity publisher will slow down and stop before hitting the
obstacle.  It will then wait a specified amount of time.  If the obstacle persists
it will send an abort signal through the segStatus message and wait until a new
set of pathSegments is published. If the obstacle is moving after it has completely
stopped it will stay in place until the obstacle starts moving for the specified
time or it moves out of the way.  If the obstacle moves out of the way the
velocity profiler will continue the segments it has.

Velocity Profiler will also stop for an E-stop message.  After the E-stop is 
disabled Velocity Profiler will continue along the specified path segments.
"""
def main():
    global RATE
    global lastVCmd
    global lastOCmd
    global obs
    global obs_dist
    global stopped
    global seg_number
    global currSegExists
    global currSeg
    global nextSegExists
    global nextSeg
    
    rospy.init_node('main')
    desVelPub = rospy.Publisher('des_vel',TwistMsg) # Steering reads this and adds steering corrections on top of the desired velocities
    segStatusPub = rospy.Publisher('seg_status', SegStatusMsg) # Lets the other nodes know what path segment the robot is currently executing
    rospy.Subscriber("motors_enabled", BoolMsg, eStopCallback) # Lets velocity profiler know the E-stop is enabled
    rospy.Subscriber("obstacles", ObstaclesMsg, obstaclesCallback) # Lets velocity profiler know where along the path there is an obstacle 
    rospy.Subscriber("cmd_vel", TwistMsg, velCmdCallback) # 
    rospy.Subscriber("path_seg", PathSegmentMsg, pathSegmentCallback)
    
    naptime = rospy.Rate(RATE)
    
    # while(not ros.Time.isValid()):
        #pass
    
    print "Entering main loop"
    
    dist = 0.0
    while(not rospy.is_shutdown()):
        if(not currSegExists):
            if(nextSegExists):
                currSegExists = True
                nextSegExists = False
                currSeg = nextSeg
            if(currSeg.seg_type == PathSegmentMsg.LINE):
                xs = currSeg.ref_point.x
                ys = currSeg.ref_point.y
                
                desired_heading = State.getYaw(currSeg.init_tan_angle)
                
                xf = xs + currSeg.seg_length*cos(desired_heading)
                yf = ys + currSeg.seg_length*sin(desired_heading)
                
                dist = sqrt(pow(xf-xs,2)+pow(yf-ys,2))
                seg_number = currSeg.seg_number
                rospy.loginfo("#%i distance: %f" %(seg_number,dist))
                straight(desVelPub,segStatusPub,dist)
                currSegExists = False
            elif(currSeg.seg_type == PathSegmentMsg.SPIN_IN_PLACE):
                seg_number = currSeg.seg_number
                dist = currSeg.seg_length
                rospy.loginfo("#%i distance: %f" %(seg_number,dist))
                turn(desVelPub,segStatusPub,dist)
                currSegExists = False
            else:
                currSegExists = False
        else:
            currSegExists = False
            vel_object = TwistMsg()
            vel_object.linear.x = 0.0
            vel_object.angular.z = 0.0
            desVelPub.publish(vel_object)
            naptime.sleep()
            continue

    
if __name__ == "__main__":
    main()
    
    