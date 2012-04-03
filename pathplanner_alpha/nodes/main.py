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

from math import cos,sin,pi
from collections import deque

RATE = 30.0

obs = ObstaclesMsg()

stopped = False

segComplete = True
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
    if(segComplete != True):
        segComplete = data.segComplete
    if(segAbort != True):
        segAbort = data.abort
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
    yawList = euler_from_quaternion(quat)
    return yawList[2]
    
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

    pathSeg = PathSegmentMsg()
    pathSeg.seg_type = PathSegmentMsg.LINE
    pathSeg.seg_number = 1
    pathSeg.seg_length = 4.2
    pathSeg.ref_point.x = 8.42
    pathSeg.ref_point.y = 15.09
    pathSeg.init_tan_angle = yawToQuat(-135.7*pi/180.0)
    pathSeg.max_speeds.linear.x = .25
    pathSeg.max_speeds.angular.z = 0.0
    pathSeg.accel_limit = .25
    pathSeg.decel_limit = .25
    pathPub.publish(pathSeg)
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

def publishRightArc(pathPub, radius):
    global RATE
    global last_seg
    global pose

    naptime = rospy.Rate(RATE)

    pathSeg = PathSegmentMsg()
    pathSeg.seg_type = PathSegmentMsg.ARC
    pathSeg.seg_number = last_seg
    pathSeg.seg_length = (pi/2)*radius
    pathSeg.ref_point.x = pose.pose.position.x + radius*cos(quatToYaw(pose.pose.orientation))
    pathSeg.ref_point.y = pose.pose.position.y + radius*sin(quatToYaw(pose.pose.orientation))
    pathSeg.init_tan_angle = pose.pose.orientation
    pathSeg.curvature = -1.0/radius
    pathSeg.max_speeds.linear.x = .25
    pathSeg.max_speeds.angular.z = .25
    pathSeg.accel_limit = .25
    pathSeg.decel_limit = .25

    pathPub.publish(pathSeg)

    naptime.sleep()

    

def publishLeftArc(pathPub, radius):
    global RATE
    global last_seg
    global pose

    naptime = rospy.Rate(RATE)

    pathSeg = PathSegmentMsg()
    pathSeg.seg_type = PathSegmentMsg.ARC
    pathSeg.seg_number = last_seg
    pathSeg.seg_length = (pi/2)*radius
    pathSeg.ref_point.x = pose.pose.position.x + radius*cos(quatToYaw(pose.pose.orientation))
    pathSeg.ref_point.y = pose.pose.position.y + radius*sin(quatToYaw(pose.pose.orientation))
    pathSeg.init_tan_angle = pose.pose.orientation
    pathSeg.curvature = 1.0/radius
    pathSeg.max_speeds.linear.x = .25
    pathSeg.max_speeds.angular.z = .25
    pathSeg.accel_limit = .25
    pathSeg.decel_limit = .25

    pathPub.publish(pathSeg)

    naptime.sleep()


def goStraight(pathPub):
    global last_seg
    global RATE
    global pose

    naptime = rospy.Rate(RATE)

    pathSeg = PathSegmentMsg()
    pathSeg.seg_type = PathSegmentMsg.LINE
    pathSeg.seg_number = last_seg
    pathSeg.seg_length = 0.5
    pathSeg.ref_point.x = pose.pose.position.x
    pathSeg.ref_point.y = pose.pose.position.y
    pathSeg.init_tan_angle = pose.pose.orientation
    
    pathPub.publish(pathSeg)

    naptime.sleep()


def checkSide(cutOff,dist):
    if(dist > cutoff):
        return True
    else:
        return False

def detour(pathPub):
    global pathStack
    global segAbort

    # publish an arc based on obstacle data
    print "In detour"

    segAbort = False

    if(obs.ping_angle < 90):
        arcRadius = obs.wall_dist_right/2.0 - 20.0
        publishLeftArc(pathPub,arcRadius)
        publishRightArc(pathPub,arcRadius)
        while not rospy.is_shutdown() and not checkSide(.6,obs.wall_dist_right):
            goStraight()
        publishRightArc(pathPub,arcRadius)
        publishLeftArc(pathPub, arcRadius)
    else:
        arcRadius = obs.wall_dist_left/2.0 - 20.0
        publishRightArc(pathPub,arcRadius)
        publishLeftArc(pathPub,arcRadius)
        while not rospy.is_shutdown() and not checkSide(.6,obs.wall_dist_left):
            goStraight()
        publishLeftArc(pathPub,arcRadius)
        publishRightArc(pathPub,arcRadius)

    

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
                publishSeg1(pathSegPub)
                publishSeg2(pathSegPub)
                publishSeg3(pathSegPub)
                publishSeg4(pathSegPub)
                publishSeg5(pathSegPub)
            elif last_seg == 2:
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
            
    

if __name__ == "__main__":
    main()
