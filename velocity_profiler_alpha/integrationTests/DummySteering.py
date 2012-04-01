#!/usr/bin/env python
'''
Created on Apr 1, 2012

@author: Devin Schwab
'''

# the only job of this file is to echo des_vel to cmd_vel

# Standard ros commands to make a node
import roslib; roslib.load_manifest('velocity_profiler_alpha');
import rospy

from geometry_msgs.msg._Twist import Twist as TwistMsg

cmdVelPub = None

def desVelCallback(des_vel):
    global cmdVelPub
    cmdVelPub.publish(des_vel)

def DummySteering():
    global cmdVelPub
    rospy.init_node('main')
    cmdVelPub = rospy.Publisher('cmd_vel',TwistMsg) 
    rospy.Subscriber("des_vel", TwistMsg, desVelCallback) 
    
    rospy.spin()

if __name__ == "__main__":
    DummySteering()