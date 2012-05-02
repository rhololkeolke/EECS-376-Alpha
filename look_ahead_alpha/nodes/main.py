#!/usr/bin/env python
import roslib; roslib.load_manifest('look_ahead_alpha')
import math
import rospy
from sensor_msgs.msg import LaserScan
from msg_alpha.msg._Obstacles import Obstacles
from msg_alpha.msg._PathList import PathList as PathListMsg


scanData=[]
RATE = 20
#BOX_WIDTH = 0.5
#BOX_HEIGHT = 1.0

ping_angle = None #the angle at which the lidar scanner picks up an obstacle
pathList = []


#Receives laser data from the base_scan topic and places the data into an array
def laserCallback(laserData):
    #copy each element in the base_scan array to scanData[]
    #the i is always within the length of the base_scan array
    obsPub = rospy.Publisher('obstacles', Obstacles)     #Data should be published to the obstacles topics using the Obstacles message type                         
    obsData = Obstacles() #initalize an Obstacle message                     
    
    #obsData.wall_dist_left = data.ranges[0]
    #obsData.wall_dist_right = data.ranges[180]
    #print len(data.ranges)
    obsPub.publish(obsData)
    
    getSegment(laserData.ranges)
#    straight(laserData.ranges)
    #print len(scanData)

def pathListCallback(pathListData):

    global pathList

    for s in pathListData.segments:

        pathList.append(s)

def getSegment(scanData):
    
    global pathList
    distanceChecked = 0.0 #current distance along the path the robot has checked
    checkLimit = 1.0 #path should only be checked up to 1m.

    while distanceChecked < 1.0:
        
        #get the length of the current path segment
        for s in pathList:
            segLength = s.seg_length
            distanceChecked += segLength
            checkSegment(scanData,segLength)

#Determine if there are obstacles along a path segment
#@param the laser scan ranges data
#@param the length of the path segment to check
def checkSegment(scanData,segLength):
    obsPub = rospy.Publisher('obstacles', Obstacles)     #Data should be published to the obstacles topics using the Obstacles message type
    obsData = Obstacles() #initalize an Obstacle message
    closestObs = 90 # the range of the lidar scanner is 80m therefore no data should be beyond this value
    global ping_angle #refernce to the global

    BOX_WIDTH = 0.5
    BOX_HEIGHT = segLength
    angleSwitch = math.atanh(BOX_WIDTH) * 180/math.pi


            #Check to see if a lidar ping is less than cos(T)/Width if so there is an obstacle
            #i is always within the bounds of the scanData array
    for x in range(len(scanData)):
        
        if(x < 90-angleSwitch or x > 90+angleSwitch):
            
            if(scanData[x] < BOX_WIDTH/math.cos((180.0-x) * math.pi/180.0)):
                
                if(scanData[x] < closestObs):
                    
                    closestObs = scanData[x]
                    ping_angle = x
                    
                elif(x > 90 - angleSwitch and x < 90 + angleSwitch):
                    
                    if(scanData[x] < BOX_HEIGHT/math.cos((x-90) * math.pi/180.0)):
                        
                        if(scanData[x] < closestObs):
                            
                            closestObs = scanData[x]
                            ping_angle = x
                            
            if(closestObs < 90):
                obsData.exists = True 
                obsData.distance = closestObs
                obsData.ping_angle = ping_angle
            else:
                obsData.exists = False
                obsData.distance = 0.0

            obsPub.publish(obsData)
    #print len(scanData)



def main():
    global BOX_WIDTH, BOX_HEIGHT

    rospy.init_node('look_ahead_main')
    '''
    # set the parameters from the launch file
    if rospy.has_param('box_width'):
        BOX_WIDTH = rospy.get_param('box_width')
    else:
        BOX_WIDTH = 0.5

    if rospy.has_param('box_height'):
        BOX_HEIGHT = rospy.get_param('box_height')
    else:
        BOX_HEIGHT = 1.0
        '''
    rospy.Subscriber("base_scan",LaserScan,laserCallback)    
    rospy.Subscriber("path",PathListMsg,pathListCallback)
    
    
    rospy.spin()

if __name__ == '__main__':
    main()
