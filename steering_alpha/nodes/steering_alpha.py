#!/usr/bin/env python
# encoding: utf-8

import roslib; roslib.load_manifest('steering_alpha')
import rospy
import numpy as np
import tf
import math as m

from msg_alpha.msg._PathSegment import PathSegment as PathSegmentMsg
#from msg_alpha.msg._Obstacles import Obstacles as ObstaclesMsg
from msg_alpha.msg._SegStatus import SegStatus as SegStatusMsg
from geometry_msgs.msg._PoseStamped import PoseStamped as PoseStampedMsg
from geometry_msgs.msg._Twist import Twist as TwistMsg
from nav_msgs.msg._Odometry import Odometry as OdometryMsg

RATE = 10
lastOdom = OdometryMsg()
lastMapPose = PoseStampedMsg()
tfl = tf.TransformListener()
desVel = TwistMsg()
nextSeg = PathSegmentMsg()
# obsExists = False
# obsDist = 0
# segNumber = 0
# nextSegExists = False
# segComplete = False
# curSegExists = False
# progressMade = 0

# def obstaclesCallback(obsData):
#     global obsExists, obsDist
#     obsExists = obsData.exists
#     obsDist = obsData.distance

# def segStatusCallback(statusData):
#     global segNumber, segComplete, curSegExists, progressMade
#     segNumber = statusData.seg_number
#     if !segComplete:
#         segComplete = statusData.segComplete
#         curSegExists = False
#         progressMade = statusData.progress_made

def odomCallback(odomData):
    global lastOdom,tfl,lastMapPose
    lastOdom = odomData
    temp = PoseStampedMsg()
    temp.pose = lastOdom.pose.pose
    temp.header = lastOdom.header
    try:
        #now lastmappose has map coords
        tfl.transformPose("map",temp,lastMapPose)
    except tf.TransformException:
        rospy.roserror("Transform Error")

def velCallback(velData):
    global desVel
    #keep updating from velocity profiler
    desVel.linear.x = velData.linear.x
    desVel.angular.z = velData.angular.z

def pathSegCallback(pathData):
    global nextSeg
    # if pathData.seg_number > nextSeg.seg_number:
        nextSeg = pathData

if __name__ == '__main__':
    main()

def normalizeToPi(inAng):
    if inAng > m.pi:
        return inAng-2*m.pi
    else if inAng < -m.pi:
        return inAng+2*m.pi

def main():
    '''
    I assume that the velocity profiler will take care of making the velocity 0
    when there is an obstacle. If not then uncomment callback subscriber,
    import etc. and make a check before publishing. I also assume that we wont
    need to keep track of how far we are in the segment. I'm not sure if this
    will mess up arcs. If arcs are messed up then set psiPathSeg = current
    desired omega. This steering only changes the heading, not the speed.
    '''
    global RATE, lastMapPose, nextSeg, desVel
    rospy.init_node('steering_alpha')
    cmdPub = rospy.Publisher('cmd_vel',TwistMsg)
    # rospy.Subscriber('obstacles',ObstaclesMsg,obstaclesCallback)
    rospy.Subscriber('odom',OdometryMsg,odomCallback)
    rospy.Subscriber('des_vel',TwistMsg,velCallback)
    rospy.Subscriber('path_seg',PathSegmentMsg,pathSegCallback)
    # rospy.Subscriber('seg_status',SegStatusMsg,segStatusCallback)
    naptime = rospy.Rate(RATE)
    cmdVel = TwistMsg()
    xyRobotCoords = np.array([lastMapPose.pose.position.x, 
                              lastMapPose.pose.position.y])
    xyStartCoords = np.array([1,0])

    #control params
    #TODO: tune these
    dThreshold = 1.0 # threshold before we correct
    kOmega = 30.0
    omegaSat = 2.0 # max omega
    dt = 0.01 #timestep

    while not rospy.is_shutdown():
        while not tfl.canTransform("map", "odom", rospy.Time.now()):
            naptime.sleep() # spin till we have some map data


        vel = desVel.linear.x #m/s
        curSeg = nextSeg
        psiRobot = tf.getYaw(lastMapPose.pose.orientation)
        psiPathSeg = tf.getYaw(curSeg.init_tan_angle)
        tHat = np.array([m.cos(psiPathSeg),m.sin(PsiPathSeg)])
        #tranform rot around z pi/2 * tHat
        nHat = np.dot(np.array([[0,-1],[1,0]]), tHat)

        d = np.subtract(xyRobotCoords, xyStartCoords)
        #convert to matrix transpose then convert back
        d = np.array(np.matrix(d).T) # the code was mat'*n_hat. I think this works
        d = np.dot(d,nHat)
        # left of path, point to path
        if d > dThreshold:
            psiDes = psiPathSeg-m.pi/2
        # too far right, point to path
        else if d < -dThreshold:
            psiDes = psiPathSeg+m.pi/2
        #offset is small, gradually parallelize
        else:
            psiDes = psiPathSeg-(m.pi/2)*d/dThreshold

        #normalize to [-pi,pi]
        psiError = normalizeToPi(psiRobot-psiDes)
        omegaCmd = -kOmega*psiError
        if omegaCmd > omegaSat
            omegaCmd = omegaSat
        else if omegaCmd < -omegaSat
            omegaCmd = -omegaSat

        psiDot = omegaCmd
        xdot = vel*m.cos(psiRobot)
        xdot = vel*m.sin(psiRobot)

        #integration
        psiRobot = psiRobot+psiDot*dt
        #normalize to [-pi,pi]
        psiRobot = normalizeToPi(psiRobot)
        xyRobotCoords = xyRobotCoords+np.array([xdot,ydot])*dt
        cmdVel = desVel
        cmdVel.angular.z = omegaCmd
        cmdPub.publish(cmdVel)
        naptime.sleep()


