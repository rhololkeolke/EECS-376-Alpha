#!/usr/bin/env python

import roslib; roslib.load_manifest('pathplanner_alpha')
import rospy

from msg_alpha.msg._PathSegment import PathSegment as PathSegmentMsg
from msg_alpha.msg._Obstacles import Obstacles as ObstaclesMsg
from msg_alpha.msg._SegStatus import SegStatus as SegStatusMsg
from geometry_msgs.msg._PoseStamped import PoseStamped as PoseStampedMsg
from std_msgs.msg._Bool import Bool as BoolMsg
from geometry_msgs.msg._Quaternion import Quaternion as QuaternionMsg

from tf.transformations import quaternion_from_euler,euler_from_quaternion

from math import cos,sin,pi,sqrt
from collections import deque

RATE = 20.0

obs = ObstaclesMsg()

stopped = False

segComplete = False
segAbort = False
last_seg = 1

pose = PoseStampedMsg()

seg_number = 0

pathStack = deque()

def eStopCallback(eStop):
    global stopped
    stopped = not eStop.data

def obstaclesCallback(data):
    global obs
    obs = data

def segStatusCallback(data):
    global segComplete
    global segAbort
    global last_seg
    if(segComplete is not True):
        segComplete = data.segComplete
    if(segAbort is not True):
        segAbort = data.abort
    if(data.seg_number != 0.0): 
        last_seg = data.seg_number

def poseCallback(poseData):
    global pose
    pose = poseData

def yawToQuat(angle):
    quatList = quaternion_from_euler(0.0,0.0,angle)
    quat = QuaternionMsg()
    quat.x = quatList[0]
    quat.y = quatList[1]
    quat.z = quatList[2]
    quat.w = quatList[3]
    return quat

def quatToYaw(quat):
    try:
        return euler_from_quaternion([quat.x,quat.y,quat.z, quat.w])[2]
    except AttributeError:
        return euler_from_quaternion(quat)[2]
    
def publishSegBlank(pathPub):
    global seg_number
    global RATE

    naptime = rospy.Rate(RATE)

    pathSeg = PathSegmentMsg()
    pathPub.publish(pathSeg)
    
    naptime.sleep()
    
def publishSeg1(pathPub):
    global seg_number
    global RATE

    naptime = rospy.Rate(RATE)

    '''
    pathSeg = PathSegmentMsg()
    pathSeg.seg_type = PathSegmentMsg.LINE
    pathSeg.seg_number = 1
    pathSeg.seg_length = 4.1
    pathSeg.ref_point.x = 8.42
    pathSeg.ref_point.y = 15.09
    pathSeg.init_tan_angle = yawToQuat(-135.7*pi/180.0)
    pathSeg.max_speeds.linear.x = .25
    pathSeg.max_speeds.angular.z = 0.0
    pathSeg.accel_limit = .25
    pathSeg.decel_limit = .25
    pathPub.publish(pathSeg)
    seg_number += 1
    '''


    rightArc = PathSegmentMsg()
    rightArc.seg_type = PathSegmentMsg.ARC
    rightArc.seg_number = 1
    rightArc.seg_length = 1.2
    rightArc.init_tan_angle = pose.pose.orientation
    rightArc.curvature = -1.0/.8
    rightArc.max_speeds.linear.x = .25
    rightArc.max_speeds.angular.z = .25
    rightArc.accel_limit = .25
    rightArc.decel_limit = .25
    pathPub.publish(rightArc)

    print "Published the right arc"
    seg_number += 1

    
    naptime.sleep()

def publishSeg2(pathPub):
    global seg_number
    global RATE

    naptime = rospy.Rate(RATE)

    pathSeg = PathSegmentMsg()
    pathSeg.seg_type = PathSegmentMsg.SPIN_IN_PLACE
    pathSeg.seg_number = 2
    pathSeg.seg_length = pi/2.0
    pathSeg.ref_point.x = 5.23
    pathSeg.ref_point.y = 11.92
    pathSeg.curvature = -1.0
    pathSeg.init_tan_angle = yawToQuat(-135.7*pi/180.0)
    pathSeg.max_speeds.linear.x = 0.0
    pathSeg.max_speeds.angular.z = .25
    pathSeg.accel_limit = .25
    pathSeg.decel_limit = .25
    pathPub.publish(pathSeg)
    seg_number +=1

    naptime.sleep()

def publishSeg3(pathPub):
    global seg_number
    global RATE

    naptime = rospy.Rate(RATE)

    pathSeg = PathSegmentMsg()
    pathSeg.seg_type = PathSegmentMsg.LINE
    pathSeg.seg_number = 3
    pathSeg.seg_length = 12.0
    pathSeg.ref_point.x = 5.45
    pathSeg.ref_point.y = 11.92
    pathSeg.init_tan_angle = yawToQuat(136.0*pi/180.0)
    pathSeg.max_speeds.linear.x = .25
    pathSeg.max_speeds.angular.z = 0.0
    pathSeg.accel_limit = .25
    pathSeg.decel_limit = .25
    pathPub.publish(pathSeg)
    seg_number += 1

    naptime.sleep()

def publishSeg4(pathPub):
    global seg_number
    global RATE

    naptime = rospy.Rate(RATE)

    pathSeg = PathSegmentMsg()
    pathSeg.seg_type = PathSegmentMsg.SPIN_IN_PLACE
    pathSeg.seg_number = 4
    pathSeg.seg_length = pi/2.0
    pathSeg.ref_point.x = 14.84
    pathSeg.ref_point.y = 3.91
    pathSeg.curvature = -1.0
    pathSeg.init_tan_angle = yawToQuat(136.0*pi/180.0)
    pathSeg.max_speeds.linear.x = 0.0
    pathSeg.max_speeds.angular.z = .25
    pathSeg.accel_limit = .25
    pathSeg.decel_limit = .25
    pathPub.publish(pathSeg)
    seg_number +=1

    naptime.sleep()

def publishSeg5(pathPub):
    global seg_number
    global RATE

    naptime = rospy.Rate(RATE)

    pathSeg = PathSegmentMsg()
    pathSeg.seg_type = PathSegmentMsg.LINE
    pathSeg.seg_number = 5
    pathSeg.seg_length = 2.0
    pathSeg.ref_point.x = -3.28
    pathSeg.ref_point.y = 20.8
    pathSeg.init_tan_angle = yawToQuat(45.22*pi/180.0)
    pathSeg.max_speeds.linear.x = .25
    pathSeg.max_speeds.angular.z = 0.0
    pathSeg.accel_limit = .25
    pathSeg.decel_limit = .25
    pathPub.publish(pathSeg)
    seg_number += 1

    naptime.sleep()

def leftRightArc(pathPub,radius):
    global RATE
    global last_seg
    global pose
    global segComplete

    naptime = rospy.Rate(RATE)
    print "left right arc"
    print "\tradius %f" % (radius)
    print "\tquatToYaw %f" % (quatToYaw(pose.pose.orientation))
    print "\tposition %s" % (pose.pose.position)

    print "Sending right arc"
    rightArc = PathSegmentMsg()
    rightArc.seg_type = PathSegmentMsg.ARC
    rightArc.seg_number = last_seg
    rightArc.seg_length = 1.2
    rightArc.init_tan_angle = pose.pose.orientation
    rightArc.curvature = -1.0/.8
    rightArc.max_speeds.linear.x = .25
    rightArc.max_speeds.angular.z = .25
    rightArc.accel_limit = .25
    rightArc.decel_limit = .25

    pathPub.publish(rightArc)
    """
    if(last_seg == 1):
        rightArc.ref_point.x = pose.pose.position.x - .3 #radius*cos(46*pi/180.0)
        rightArc.ref_point.y = pose.pose.position.y + .3 #radius*sin(46*pi/180.0)
    elif(last_seg == 3):
        rightArc.ref_point.x = pose.pose.position.x + .1 #radius*cos(46*pi/180.0)
        rightArc.ref_point.y = pose.pose.position.y + .1 #radius*sin(46*pi/180.0)

    print "\tright arc ref_point %s" % rightArc.ref_point
    pathPub.publish(rightArc)
    while not rospy.is_shutdown() and not segComplete:
        naptime.sleep()
    """
    straight = PathSegmentMsg()
    straight.seg_type = PathSegmentMsg.LINE
    straight.seg_number = last_seg
    straight.seg_length = 1
    straight.ref_point.x = pose.pose.position.x
    straight.ref_point.y = pose.pose.position.y
    straight.init_tan_angle = yawToQuat(quatToYaw(pose.pose.orientation) + 5)
    straight.curvature = 0.0
    straight.max_speeds.linear.x = .25
    straight.accel_limit = .25
    straight.decel_limit = .25



    leftArc = PathSegmentMsg()
    leftArc.seg_type = PathSegmentMsg.ARC
    leftArc.seg_number = last_seg
    leftArc.seg_length = 1.25
    leftArc.init_tan_angle = pose.pose.orientation
    leftArc.curvature = 1.0/.8
    leftArc.max_speeds.linear.x = .25
    leftArc.max_speeds.angular.z = .25
    leftArc.accel_limit = .25
    leftArc.decel_limit = .25
    """
    if(last_seg == 1):
        leftArc.ref_point.x = pose.pose.position.x + .3 #(pose.pose.position.x + radius*cos(quatToYaw(pose.pose.orientation))) - radius*cos(46*pi/180.0)
        leftArc.ref_point.y = pose.pose.position.y - .3 #(pose.pose.position.y + radius*sin(quatToYaw(pose.pose.orientation))) + radius*sin(46*pi/180.0)
    elif(last_seg == 3):
        leftArc.ref_point.x = (pose.pose.position.x + radius*cos(quatToYaw(pose.pose.orientation))) + radius*cos(46*pi/180.0)
        leftArc.ref_point.y = (pose.pose.position.y + radius*sin(quatToYaw(pose.pose.orientation))) + radius*sin(46*pi/180.0)
    """
    print "Sending leftArc"
    pathPub.publish(leftArc)
    naptime.sleep()

    segComplete = False
    maxIter = 50
    count = 1
    # hold here so the coordinates for the straight are correct
    while not rospy.is_shutdown() and not segComplete:
        if count > maxIter:
            break # for some reason the segment didn't complete, or the message was missed
        naptime.sleep()


    print "Sending straight"
    pathPub.publish(straight)
    naptime.sleep()

    print "sending sleep"
    pathPub.publish(leftArc)
    naptime.sleep()
    
    print "rightaArc"
    pathPub.publish(rightArc)
    naptime.sleep()

def rightLeftArc(pathPub,radius):
    global RATE
    global last_seg
    global pose

    naptime = rospy.Rate(RATE)

    print "rightLeftArc"
    print "\tradius %f" % (radius)
    print "\tquatToYaw %f" % (quatToYaw(pose.pose.orientation))
    print "\tposition %s" % (pose.pose.position)

    """
    if(last_seg == 1):
        rightArc.ref_point.x = pose.pose.position.x + radius*cos(46*pi/180.0) 
        rightArc.ref_point.y = pose.pose.position.y - radius*sin(46*pi/180.0)
    elif(last_seg == 3):
        rightArc.ref_point.x = pose.pose.position.x - radius*cos(46*pi/180.0)
        rightArc.ref_point.y = pose.pose.position.y - radius*sin(46*pi/180.0)
    """

    leftArc = PathSegmentMsg()
    leftArc.seg_type = PathSegmentMsg.ARC
    leftArc.seg_number = last_seg
    leftArc.seg_length = 1.0
    leftArc.init_tan_angle = pose.pose.orientation
    leftArc.curvature = 1.0/.8
    leftArc.max_speeds.linear.x = .25
    leftArc.max_speeds.angular.z = .25
    leftArc.accel_limit = .25
    leftArc.decel_limit = .25

    rightArc = PathSegmentMsg()
    rightArc.seg_type = PathSegmentMsg.ARC
    rightArc.seg_number = last_seg
    rightArc.seg_length = 1.0
    rightArc.init_tan_angle = pose.pose.orientation
    rightArc.curvature = -1.0/.8
    rightArc.max_speeds.linear.x = .25
    rightArc.max_speeds.angular.z = .25
    rightArc.accel_limit = .25
    rightArc.decel_limit = .25

    """
    if(last_seg == 1):
        rightArc.ref_point.x = (pose.pose.position.x + radius*cos(quatToYaw(pose.pose.orientation))) + radius*cos(46*pi/180.0)
        rightArc.ref_point.y = (pose.pose.position.y + radius*sin(quatToYaw(pose.pose.orientation))) + radius*sin(46*pi/180.0)
    elif(last_seg == 3):
        rightArc.ref_point.x = (pose.pose.position.x + radius*cos(quatToYaw(pose.pose.orientation))) - radius*cos(46*pi/180.0)
        rightArc.ref_point.y = (pose.pose.position.y + radius*sin(quatToYaw(pose.pose.orientation)))- radius*sin(46*pi/180.0)
    """

    pathPub.publish(leftArc)
    naptime.sleep()
    pathPub.publish(rightArc)
    naptime.sleep()

def goStraight(pathPub):
    global last_seg
    global RATE
    global pose

    naptime = rospy.Rate(RATE)
    print "In the go straight method"

    pathSeg = PathSegmentMsg()
    pathSeg.seg_type = PathSegmentMsg.LINE
    pathSeg.seg_number = last_seg
    pathSeg.seg_length = .5
    pathSeg.ref_point.x = pose.pose.position.x
    pathSeg.ref_point.y = pose.pose.position.y
    pathSeg.init_tan_angle = pose.pose.orientation
    pathSeg.max_speeds.linear.x = .25
    pathSeg.max_speeds.angular.z = .25
    pathSeg.accel_limit = .25
    pathSeg.decel_limit = .25

    pathPub.publish(pathSeg)

    naptime.sleep()


def checkSide(cutoff,dist):
    if(dist > cutoff):
        return True
    else:
        return False

def detour(pathPub):
    global obs
    global segAbort
    global segComplete
    global RATE

    # publish an arc based on obstacle data
    print "In detour"

    segAbort = False
    naptime = rospy.Rate(RATE)

    if(obs.ping_angle < 90):
        print "ping less than 90"
        arcRadius = .6 #obs.wall_dist_left/2.0 - .20
        print "arcRadius: %f" % (arcRadius)
        rightLeftArc(pathPub,arcRadius)
        #while not segComplete:
        #    naptime.sleep()
        #goStraight(pathPub)
        #leftRightArc(pathPub,arcRadius)
    else:
        print "ping greater than 90"
        arcRadius = .6 #obs.wall_dist_right/2.0 -.2
        leftRightArc(pathPub,arcRadius)
        #while not segComplete:
        #    naptime.sleep()
        #goStraight(pathPub)
        #rightLeftArc(pathPub,arcRadius)

    

def main():
    global RATE
    global obs
    global segStatus
    global pose
    global pathStack
    global segAbort
    global segComplete
    global last_seg

    rospy.init_node('path_planner_alpha')
    pathSegPub = rospy.Publisher('path_seg',PathSegmentMsg)
    rospy.Subscriber('seg_status', SegStatusMsg, segStatusCallback)
    rospy.Subscriber('motors_enabled', BoolMsg, eStopCallback)
    rospy.Subscriber('obstacles', ObstaclesMsg, obstaclesCallback)
    rospy.Subscriber('map_pos', PoseStampedMsg, poseCallback)

    naptime = rospy.Rate(RATE)

    print "Entering main loop"

    publishSegBlank(pathSegPub)
    publishSeg1(pathSegPub)
    publishSeg2(pathSegPub)
    publishSeg3(pathSegPub)
    publishSeg4(pathSegPub)
    publishSeg5(pathSegPub)
    

    while not rospy.is_shutdown():
        
        if segAbort:
            detour(pathSegPub)
            if last_seg == 1:
                print "last_seg == 1"
                publishSeg1(pathSegPub)
                publishSeg2(pathSegPub)
                publishSeg3(pathSegPub)
                publishSeg4(pathSegPub)
                publishSeg5(pathSegPub)
            elif last_seg == 2:
                print "last_seg == 2"
                publishSeg2(pathSegPub)
                publishSeg3(pathSegPub)
                publishSeg4(pathSegPub)
                publishSeg5(pathSegPub)
            
                
            elif last_seg == 3:
               publishSeg3(pathSegPub)
               publishSeg4(pathSegPub)
               publishSeg5(pathSegPub)
            elif last_seg == 4:
                publishSeg4(pathSegPub)
                publishSeg5(pathSegPub)
            elif last_seg == 5:
                publishSeg5(pathSegPub)
        


            naptime.sleep()
           
    

if __name__ == "__main__":
    main()
