#!/usr/bin/env python

'''
This program will take in a point of the closest centroid of orange.
The robot will then steer to that point.  If the strap is lost
the robot will spin in place until more orange is detected.
However, it will ignore orange centroids detected when the robot
is pi radians away from the orientation it started to spin in place at.
This is to prevent the robot from following the same strap back to the beginning.
'''

import roslib; roslib.load_manifest('velocity_profiler_alpha')
import rospy

from msg_alpha.msg._CentroidPoints import CentroidPoints as CentroidPointsMsg
from geometry_msgs.msg._Quaternion import Quaternion as QuaternionMsg
from geometry_msgs.msg._PoseStamped import PoseStamped as PoseStampedMsg
from geometry_msgs.msg._Point import Point as PointMsg
from geometry_msgs.msg._Twist import Twist as TwistMsg
from tf.transformations import quaternion_from_euler,euler_from_quaternion

from math import atan2,pi

# set the rate the node runs at
RATE = 20.0

# this will store a Rate instance to keep the node running at the specified RATE
naptime = None # this will be initialized first thing in main

# Current point to steer to
currPoint = None

# pose data
position = PointMsg()
orientation = QuaternionMsg()

def poseCallback(pose):
    '''
    Updates the robot's best estimate on position and orientation
    '''
    global position
    global orientation
    position = pose.pose.position
    orientation = pose.pose.orientation

def centroidPointCallback(data):
    '''
    Remembers the point specified by the image processing node
    '''
    global currPoint
    # if the point is (0,0) then no centroid was found
    # in reality there should be a better way of signaling this
    if(data.point.x == 0 and data.point.y == 0):
        currPoint = None
    else:
        currPoint = data.point

def getYaw(quat):
    try:
        return euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])[2]
    except AttributeError:
        return euler_from_quaternion(quat)[2]


def main():
    '''
    The main function that is executed while the node is running
    '''
    global RATE, naptime

    rospy.init_node('steering_alpha_main')
    naptime = rospy.Rate(RATE)

    cmdPub = rospy.Publisher('cmd_vel',TwistMsg)
    
    rospy.Subscriber('map_pos', PoseStampedMsg, poseCallback)
    rospy.Subscriber('centroid_point', CentroidPointsMsg, centroidPointCallback)

    print "Entering main loop"
    
    Kd = 0.5
    Ktheta = 1.0

    while(not rospy.is_shutdown()):
        if(currPoint is None):
            # For now publish stop messages when no point is detected
            # Eventually this will be the spin routine
            cmdPub.publish(TwistMsg())
            naptime.sleep()
            continue
        
        xVec = currPoint.x-position.x
        yVec = currPoint.y-position.y

        actPsi = getYaw(orientation)
        desPsi = atan2(yVec,xVec)

        dTheta = (desPsi - actPsi) % (2*pi)
        if(dTheta > pi):
            dTheta = dTheta - 2*pi

        cmd_vel = TwistMsg()
        cmd_vel.angular.z = Ktheta*dTheta
        cmd_vel.linear.x = .25 # Eventually this will actually accelerate and decelerate

        cmdPub.publish(cmd_vel)
        naptime.sleep()

if __name__ == "__main__":
    main()
