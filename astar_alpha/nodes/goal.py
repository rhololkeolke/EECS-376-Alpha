#!/usr/bin/env python
import roslib; roslib.load_manifest('astar_alpha')
from geometry_msgs.msg._Point import Point as PointMsg

import rospy

def goalPublisher():
    pub = rospy.Publisher('goal', PointMsg)
    rospy.init_node('n')
    while not rospy.is_shutdown():
        PointMsg.x = 
        PointMsg.y = 
        PointMsg.z = 
        pub.publish(PointMsg)
        rospy.sleep(1.0)


if __name__ == '__main__':
    try:
        goalPublisher()
    except rospy.ROSInterruptException: pass
