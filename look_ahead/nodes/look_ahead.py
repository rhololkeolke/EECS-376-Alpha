#!/usr/bin/env python
import roslib; roslib.load_manifest('look_ahead')
import math
import rospy
from sensor_msgs.msg import LaserScan
from msg_alpha.msg._Obstacles import Obstacles





scanData=[]
BOX_WIDTH = 0.5
BOX_HEIGHT = 1.0
angleSwitch = math.atanh(BOX_WIDTH) * 180/math.pi
ping_angle = None #the angle at which the lidar scanner picks up an obstacle

#Receives laser data from the base_scan topic and places the data into an array
def laserCallback(data):
    #copy each element in the base_scan array to scanData[]
    #the i is always within the length of the base_scan array
    obsPub = rospy.Publisher('obstacles', Obstacles)     #Data should be published to the obstacles topics using the Obstacles message type                         
    obsData = Obstacles() #initalize an Obstacle message                     
    
    #obsData.wall_dist_left = data.ranges[0]
    #obsData.wall_dist_right = data.ranges[180]
    #print len(data.ranges)
    obsPub.publish(obsData)
    
    straight(data.ranges)
    #print len(scanData)

#Determine if there are obstacles along a straight path        
def straight(scanData):
    obsPub = rospy.Publisher('obstacles', Obstacles)     #Data should be published to the obstacles topics using the Obstacles message type
    obsData = Obstacles() #initalize an Obstacle message
    closestObs = 90 # the range of the lidar scanner is 80m therefore no data should be beyond this value
    global ping_angle #refernce to the global


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

    obsData.wall_dist_right = scanData[0]
    obsData.wall_dist_left = scanData[180]

        
    obsPub.publish(obsData)
    #print len(scanData)

def main():
    
    rospy.init_node('n')
    r = rospy.Rate(100)
    rospy.Subscriber("base_scan",LaserScan,laserCallback)    
    rospy.spin()

if __name__ == '__main__':

    main()

