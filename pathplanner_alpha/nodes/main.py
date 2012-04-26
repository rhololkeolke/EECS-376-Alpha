import roslib; roslib.load_manifest('pathplanner_alpha')
import rospy
import math as m

from msg_alpha.msg._PathSegment import PathSegment as PathSegmentMsg
from msg_alpha.msg._Obstacles import Obstacles as ObstaclesMsg
from msg_alpha.msg._SegStatus import SegStatus as SegStatusMsg
from msg_alpha.msg._PathList import PathList as PathListMsg
from geometry_msgs.msg._PoseStamped import PoseStamped as PoseStampedMsg
from std_msgs.msg._Bool import Bool as BoolMsg
from geometry_msgs.msg._Quaternion import Quaternion as QuaternionMsg

from tf.transformations import quaternion_from_euler,euler_from_quaternion

from math import cos,sin,pi,sqrt
from collections import deque

obs = ObstaclesMsg()

stopped = False

segComplete = False
segAbort = False
last_seg = 1

pose = PoseStampedMsg()

segNumber = 0

pathList = []
pastPoint = PointMsg()
currentPoint = PointMsg()
#futurePoint = PointMsg()

def eStopCallback(eStop):
	global stopped
	stoppped = not eStop.data

def obstaclesCallback(data):
    global obs
    obs = data

def segStatusCallback(data):
    global segComplete
    global segAbort
    global last_seg
    if(segComplete is not True):
        segComplete = data.segComplete
    if(segAbort is not True):
        segAbort = data.abort
    if(data.seg_number != 0.0): 
        last_seg = data.seg_number

def poseCallback(poseData):
    global pose
    pose = poseData

def pointListCallback(data):
    global desPoints[]
    data.cells = desPoints[]

def yawToQuat(angle):
    quatList = quaternion_from_euler(0.0,0.0,angle)
    quat = QuaternionMsg()
    quat.x = quatList[0]
    quat.y = quatList[1]
    quat.z = quatList[2]
    quat.w = quatList[3]
    return quat

def quatToYaw(quat):
    try:
        return euler_from_quaternion([quat.x,quat.y,quat.z, quat.w])[2]
    except AttributeError:
        return euler_from_quaternion(quat)[2]

def publishSegBlank(pathPub)
    global seg_number
    global RATE

    naptime = rospy.Rate(RATE)

    pathSeg = PathSegmentMsg()
    pathPub.publish(pathSeg)
    
    naptime.sleep()

def addSegToList(PathSegmentMsg())

    pathSeg = PathSegmentMsg()

    pathSeg.seg_number = segNumber+1
    pathSeg.max_speeds = .25
    pathSeg.min_speeds = 0
    pathSeg.accel_limit = .125
    pathSeg.decel_limit = -.125
    #Maybe?
    pathSeg.curvature = 0
    
    pathList.append(pathSeg)

    naptime.sleep()


def main():
	rospy.init_node()
	rospy.init_node('path_planner_alpha')
    pathSegPub = rospy.Publisher('path_seg',PathSegmentMsg)
    rospy.Subscriber('seg_status', SegStatusMsg, segStatusCallback)
    rospy.Subscriber('motors_enabled', BoolMsg, eStopCallback)
    rospy.Subscriber('obstacles', ObstaclesMsg, obstaclesCallback)
    rospy.Subscriber('map_pos', PoseStampedMsg, poseCallback)
    rospy.Subscriber('pointList', PathListMsg, pointListCallback)

    pathSeg = PathSegmentMsg()

    while not rospy.is_shutdown():

        if(segAbort):
            segNumber = 0
            pathList = []

        if(len(desPoints) >= segNumber+1):
            
            pathSeg.init_tan_angle = m.atan2((desPoints[segNumber+1].y-desPoints[segNumber].y),(desPoints[segNumber+1].x-desPoints[segNumber].x))

            #if(theta > m.pi/6):
                #SPIN TO NEW ANGLES
            #   addSegToList()

            #Calculate the length of the given segment
            #Add angle
            xDist = m.pow((desPoints[segNumber].x - desPoints[segNumber+1].x),2)
            yDist = m.pow((desPoints[segNumber].y - desPoints[segNumber+1].y),2)
            pathSeg.segLength = m.sqrt(xDist + yDist)

            addSegToList()

        if(len(desPoints) == len(pathList)):
            pathSegPub.publish(pathList)



    naptime.sleep()


if __name__ == "__main__":
    main()


#Add code to check and see if path has changed
#Make changes to list if the possible places has changed
