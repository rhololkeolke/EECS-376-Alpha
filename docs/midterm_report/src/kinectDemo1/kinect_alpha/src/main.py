#!/usr/bin/env python

import roslib; roslib.load_manifest('kinect_alpha')
import rospy

from msg_alpha.msg._BlobDistance import BlobDistance
from geometry_msgs.msg._Twist import Twist as TwistMsg

# add some deadzone to the middle of the image to prevent jitter
THRESHHOLD = 320
distance = 0

def distanceCallback(dist):
	global distance
	distance = dist.dist

def main():
	# standard ros code
	rospy.init_node('kinect_move_alpha')
	desVelPub = rospy.Publisher('cmd_vel', TwistMsg)

	rospy.Subscriber("blob_dist", BlobDistance, distanceCallback)

	vel_cmd = TwistMsg()

	naptime = rospy.Rate(20.0)
	
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
		naptime.sleep()


if __name__ == "__main__":
    main()
