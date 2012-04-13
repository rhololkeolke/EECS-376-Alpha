#!/usr/bin/env python
# encoding: utf-8

import roslib; roslib.load_manifest('steering_alpha')
import rospy
import numpy as np
import tf
import math

from msg_alpha.msg._PathSegment import PathSegment as PathSegmentMsg
from msg_alpha.msg._Obstacles import Obstacles as ObstaclesMsg
from msg_alpha.msg._SegStatus import SegStatus as SegStatusMsg
from geometry_msgs.msg._PoseStamped import PoseStamped as PoseStampedMsg
from geometry_msgs.msg._Twist import Twist as TwistMsg
from nav_msgs.msg._Odometry import Odometry as OdometryMsg

RATE = 10
obsExists = False
obsDist = 0
lastOdom = OdometryMsg()
lastMapPose = PoseStampedMsg()
tfl = tf.TransformListener()
desVel = TwistMsg()
nextSegExists = False
nextSeg = PathSegmentMsg()
segNumber = 0
segComplete = False
curSegExists = False
progressMade = 0

def obstaclesCallback(obsData):
    global obsExists, obsDist
    obsExists = obsData.exists
    obsDist = obsData.distance

def odomCallback(odomData):
    global lastOdom,tfl,lastMapPose
    lastOdom = odomData
    temp = PoseStampedMsg()
    temp.pose = lastOdom.pose.pose
    temp.header = lastOdom.header
    try:
        tfl.transformPose("map",temp,lastMapPose)
    except tf.TransformException:
        rospy.roserror("Transform Error")

def velCallback(velData):
    global desVel
    desVel.linear.x = velData.linear.x
    desVel.angular.z = velData.angular.z

def pathSegCallback(pathData):
    global nextSegExists, nextSeg
    if pathData.seg_number > nextSeg.seg_number:
        nextSegExists = True
        nextSeg = pathData

def segStatusCallback(statusData):
    global segNumber, segComplete, curSegExists, progressMade
    segNumber = statusData.seg_number
    if !segComplete:
        segComplete = statusData.segComplete
        curSegExists = False
        progressMade = statusData.progress_made

if __name__ == '__main__':
    main()

def normalizeAngle(offset, threshold, angle):
    # left of path, point to path
    if offset > threshold:
        return angle-math.pi/2
    # too far right, point to path
    else if offset < -threshold:
        return angle+math.pi/2
    #offset is small, gradually parallelize
    else:
        return angle-(math.pi/2)*offset/threshold

def main():
    global RATE, lastMapPose, nextSeg
    rospy.init_node('steering_alpha')
    rospy.Publisher('cmd_vel',TwistMsg)
    rospy.Subscriber('obstacles',ObstaclesMsg,obstaclesCallback)
    rospy.Subscriber('odom',OdometryMsg,odomCallback)
    rospy.Subscriber('des_vel',TwistMsg,velCallback)
    rospy.Subscriber('path_seg',PathSegmentMsg,pathSegCallback)
    rospy.Subscriber('seg_status',SegStatusMsg,segStatusCallback)
    naptime = rospy.Rate(RATE)

    while not rospy.is_shutdown():
        while not tfl.canTransform("map", "odom", rospy.Time.now()):
            pass # spin till we have some map data

        #control params
        #TODO: tune these
        dThreshold = 1.0
        kOmega = 30.0
        omegaSat = 2.0

        curSeg = nextSeg
        xyStartCoords = np.array([1,0])
        xyRobotCoords = np.array([lastMapPose.pose.position.x, 
                               lastMapPose.pose.position.y])
        psiRobot = tf.getYaw(lastMapPose.pose.orientation)
        psiPathSeg = tf.getYaw(curSeg.init_tan_angle)
        tHat = np.array([math.cos(psiPathSeg),math.sin(PsiPathSeg)])
        #tranform rot around z pi/2 * tHat
        nHat = np.dot(np.array([[0,-1],[1,0]]), tHat)
        

        d = np.subtract(xyRobotCoords, xyStartCoords)
        #convert to matrix transpose then convert back
        d = np.array(np.matrix(d).T) # the code was mat'*n_hat. I think this works
        d = np.dot(d,nHat)
        psiDes = normalizeAngle(d,dThreshold,psiPathSeg)
        psiError = psiRobot-psiDes


