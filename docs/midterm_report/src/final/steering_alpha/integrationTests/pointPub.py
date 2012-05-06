#!/usr/bin/env python

'''
This program will be used to test the strap follower program
'''

import roslib; roslib.load_manifest('steering_alpha')
import rospy

from msg_alpha.msg._CentroidPoints import CentroidPoints as CentroidPointsMsg
from geometry_msgs.msg._Point import Point as PointMsg

from math import cos,sin,tan,pi,sqrt

# set the rate the node runs at
RATE = 20.0

# this will store a Rate instance to keep the node running at the specified RATE
naptime = None # this will be initialized first thing in main

def main():
    global RATE, naptime

    rospy.init_node('point_pub')
    naptime = rospy.Rate(RATE)

    centroidPub = rospy.Publisher('centroid_point', CentroidPointsMsg)

    print "Entering main loop"
    
    desX = raw_input("Enter the desired x coordinate")
    desY = raw_input("Etner the desired y coordinate")

    desX = float(desX)
    desY = float(desY)
    
    centroid = CentroidPointsMsg()
    centroid.point.x = desX
    centroid.point.y = desY

    while(not rospy.is_shutdown()):
        centroidPub.publish(centroid)
        naptime.sleep()

if __name__ == "__main__":
    main()
