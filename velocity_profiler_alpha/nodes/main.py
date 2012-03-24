#!/usr/bin/evn python
'''
Created on Mar 22, 2012

@author: Devin Schwab
'''

# Standard ros commands to make a node
import roslib; roslib.load_manifest('velocity_profiler_alpha');
import rospy

# message data types
from geometry_msgs.msg._Twist import Twist as TwistMsg
from std_msgs.msg._Bool import Bool as BoolMsg
from msg_alpha.msg._PathSegment import PathSegment as PathSegmentMsg 
from msg_alpha.msg._Obstacles import Obstacles as ObstaclesMsg 
from msg_alpha.msg._SegStatus import SegStatus as SegStatusMsg 

def eStopCallback(data):
    pass

def obstaclesCallback(data):
    pass

def velCmdCallback(data):
    pass

def pathSegmentCallback(data):
    pass



"""
This function is responsible for velocity profiling.  

In general velocity profiler takes in path segments from path publisher
and creates a trajectory based on the segments. The trajectory will smooth
the segments into each other based on the acceleration and velocity 
constraints of the current path segment and the future line segments. 
The velocity and acceleration constraints of the path segment are assumed 
to be slow enough to not cause wheel slip.

Velocity profiler is also in charge of keeping track of how far along the
trajectory the robot actually is.  It creates velocity commands based on where
along the trajectory the robot is. It publishes these commands to the steering node.

Velocity profiler reads in the Obstacles message and checks for obstacles.  If an
obstacle is detected velocity publisher will slow down and stop before hitting the
obstacle.  It will then wait a specified amount of time.  If the obstacle persists
it will send an abort signal through the segStatus message and wait until a new
set of pathSegments is published. If the obstacle is moving after it has completely
stopped it will stay in place until the obstacle starts moving for the specified
time or it moves out of the way.  If the obstacle moves out of the way the
velocity profiler will continue the segments it has.

Velocity Profiler will also stop for an E-stop message.  After the E-stop is 
disabled Velocity Profiler will continue along the specified path segments.
"""
def main():
    rospy.init_node('main')
    desVelPub = rospy.Publisher('des_vel',TwistMsg) # Steering reads this and adds steering corrections on top of the desired velocities
    segStatusPub = rospy.Publisher('seg_status', SegStatusMsg) # Lets the other nodes know what path segment the robot is currently executing
    rospy.Subscriber("motors_enabled", BoolMsg, eStopCallback) # Lets velocity profiler know the E-stop is enabled
    rospy.Subscriber("obstacles", ObstaclesMsg, obstaclesCallback) # Lets velocity profiler know where along the path there is an obstacle 
    rospy.Subscriber("cmd_vel", TwistMsg, velCmdCallback) # 
    rospy.Subscriber("path_seg", PathSegmentMsg, pathSegmentCallback)
    
    print "About to spin"
    rospy.spin()
    
if __name__ == "__main__":
    main()
    
    