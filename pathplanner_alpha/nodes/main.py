import roslib; roslib.load_manifest('pathplanner_alpha')
import rospy
import math as m

from msg_alpha.msg._PathSegment import PathSegment as PathSegmentMsg
from msg_alpha.msg._Obstacles import Obstacles as ObstaclesMsg
from msg_alpha.msg._SegStatus import SegStatus as SegStatusMsg
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

seg_number = 0

pathStack = deque()

pastPoint = PointMsg()
currentPoint = PointMsg()
futurePoint = PointMsg()

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

#TODO: 
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
    global seg_number

    pathSeg = PathSegmentMsg()

    #Takes in a pathSeg and adds it to the list that will be published

    naptime.sleep()


def main():
	rospy.init_node()
	rospy.init_node('path_planner_alpha')
    pathSegPub = rospy.Publisher('path_seg',PathSegmentMsg)
    rospy.Subscriber('seg_status', SegStatusMsg, segStatusCallback)
    rospy.Subscriber('motors_enabled', BoolMsg, eStopCallback)
    rospy.Subscriber('obstacles', ObstaclesMsg, obstaclesCallback)
    rospy.Subscriber('map_pos', PoseStampedMsg, poseCallback)
    rospy.Subscriber('pointList', )

    segNumber = 0
    isTurn = False
    pathSeg = PathSegmentMsg()



    while not rospy.is_shutdown():
        if stopped:
            stopForEstop(desVelPub, segStatusPub)

        if(segAbort):
            segNumber = 0

        if(desPoints[segNumber] is not None):
            
            if(desPoints[segNumber] is not 0):
            #Calculate the angle between the new line and the old line
            #to determine if a spin in place is needed between the two
            
            oldM = ((desPoints[segNumber].x-desPoints[segNumber+1].x)/(desPoints[segNumber].y-desPoints[segNumber+1].y)) 
            newM = ((desPoints[segNumber+1].x-desPoints[segNumber+2].x)/(desPoints[segNumber+1].y-desPoints[segNumber+2].y))
            theta = m.atan2((oldM-newM)/(1+oldM*newM))

            if(theta > m.pi/6):
                #SPIN TO NEW ANGLES
                addSegToList()


            else:
                #Calculate the length of the given segment
                xDist = m.pow((desPoints[segNumber].x - desPoints[segNumber+1].x),2)
                yDist = m.pow((desPoints[segNumber].y - desPoints[segNumber+1].y),2)
                pathSeg.segLength = m.sqrt(xDist + yDist)
                addSegToList()


 

    naptime.sleep()


if __name__ == "__main__":
    main()