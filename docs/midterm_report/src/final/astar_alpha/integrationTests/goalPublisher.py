#!/usr/bin/env python

import roslib; roslib.load_manifest('astar_alpha')
import rospy

from msg_alpha.msg._Goal import Goal as GoalMsg
from geometry_msgs.msg._Point import Point as PointMsg

RATE = 20.0

def main(x=None,y=None):
    global goal

    rospy.init_node('astar_alpha_goal_publisher')
    
    goalPub = rospy.Publisher('goal_point',GoalMsg)

    naptime = rospy.Rate(RATE)

    goal = GoalMsg()

    goal.new = True

    
    if(x is None or y is None):
        goal.none = True
    else:
        goal_point = PointMsg()
        goal_point.x = x
        goal_point.y = y

        goal.goal = goal_point
    
    while not rospy.is_shutdown():
        goalPub.publish(goal)
        goal.new = False

        naptime.sleep()

if __name__ == "__main__":
    import sys
    
    if(len(sys.argv) == 3):
        try:
            x = float(sys.argv[1])
            y = float(sys.argv[2])
            main(x,y)
        except ValueError:
            print "Non floats detected."
            print "Publishing blank path segments"
            
            main()
    else:
        print "Non floats detected."
        print "Publishing blank path segments"
        
        main()
