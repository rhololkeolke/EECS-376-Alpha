#!/usr/bin/env python

import roslib; roslib.load_manifest('kinect_alpha')
import rospy

from msg_alpha.msg._BlobDistance import dist as dist
from geomerty_msg.msg._Twist import Twist as Twist

THRESHHOLD = 320
distance = 0

def distanceCallback(dist)
	global distance
	distance = dist

def main()

	rospy.init_node('kinect_move_alpha')
	desVelPub = rospy.Publisher('cmd_vel', TwistMsg)

	vel_cmd = TwistMsg()
	
	while(not rospy.is_shutdown()):
		#Max speed is .25	
		if(distance > THRESHHOLD + 20):
			vel_cmd.angular.z = -.25
			pass	
		elif(distance<THRESHHOLD-20):
			vel_cmd.angular.z = .25
		else:
			vel_cmd.angular.z = 0

		desVelPub.publish(vel_cmd)


if __name__ == "__main__":
    main()