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
from Queue import Queue
from Queue import Empty as QueueEmpty

RATE = 30.0

obs = ObstaclesMsg()

stopped = False

segStatus = SegStatusMsg()

pose = PoseStampedMsg()

seg_number = 0

def eStopCallback(eStop):
    global stopped
    stopped = not eStop.data

def obstaclesCallback(data):
    global obs
    obs = data

def segStatusCallback(data):
    global segStatus
    segStatus = data

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
    
def publishSegments(pathPub):
    global seg_number
    global RATE

    naptime = rospy.Rate(RATE)

    pathSeg = PathSegmentMsg()
    pathSeg.seg_type = PathSegmentMsg.LINE
    pathSeg.seg_number = seg_number
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

    print pathSeg

    naptime.sleep()

    pathSeg = PathSegmentMsg()
    pathSeg.seg_type = PathSegmentMsg.SPIN_IN_PLACE
    pathSeg.seg_number = seg_number
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

    print pathSeg
    naptime.sleep()

    pathSeg = PathSegmentMsg()
    pathSeg.seg_type = PathSegmentMsg.LINE
    pathSeg.seg_number = seg_number
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

    pathSeg = PathSegmentMsg()
    pathSeg.seg_type = PathSegmentMsg.SPIN_IN_PLACE
    pathSeg.seg_number = seg_number
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

    pathSeg = PathSegmentMsg()
    pathSeg.seg_type = PathSegmentMsg.LINE
    pathSeg.seg_number = seg_number
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

def main():
    global RATE
    global obs
    global segStatus
    global pose

    rospy.init_node('path_planner_alpha')
    pathSegPub = rospy.Publisher('path_seg',PathSegmentMsg)
    rospy.Subscriber('seg_status', SegStatusMsg, segStatusCallback)
    rospy.Subscriber('motors_enabled', BoolMsg, eStopCallback)
    rospy.Subscriber('obstacles', ObstaclesMsg, obstaclesCallback)
    rospy.Subscriber('map_pos', PoseStampedMsg, poseCallback)

    naptime = rospy.Rate(RATE)

    print "Entering main loop"
    
    publishSegments(pathSegPub)

if __name__ == "__main__":
    main()
