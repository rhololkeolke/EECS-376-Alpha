#!/usr/bin/env python

import roslib; roslib.load_manifest('magnet_test')
import rospy
from phidgets.srv._interface_kit import interface_kit

from msg_alpha.msg._MagEngaged import MagEngaged as MagnetMsg

engaged = None

toggle_magnet = None



def magCallback(data):
	global engaged

	engaged = data.mag_engaged

def main():
	global toggle_magnet


	rospy.init_node('magnet_toggle')

	rospy.Subscriber('magnet_toggle', MagnetMsg, magCallback)
	
	rospy.wait_for_service('interface_kit')

	toggle_magnet = rospy.ServiceProxy('interface_kit', interface_kit)

	while not rospy.is_shutdown():
		if engaged is True:
			toggle_magnet(6,1,1)
		else:
			toggle_magnet(6,1,0)


if __name__ == "__main__":
    main()