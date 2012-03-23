#!/usr/bin/env python
import roslib; roslib.load_manifest('velocity_profiler')
import math
import rospy
from sensor_msgs.msg import LaserScan
from velocity_profiler.msg import Obstacles





scanData=[]
BOX_WIDTH = 0.5
BOX_HEIGHT = 1.0
angleSwitch = math.atanh(BOX_WIDTH) * 180/math.pi



def laserCallback(data):
    for i in range(len(data.ranges)):
        scanData.append(data.ranges[i])
        
def straight():
    obsPub = rospy.Publisher('obstacles', Obstacles)
    obsData = Obstacles()
    closestObs = 90
    int ping_angle

    for x in range(len(scanData)):
        
        if(x < 90-angleSwitch or x > 90+angleSwitch):

            if(scanData[x] < BOX_WIDTH/math.cos((180.0-x) * math.pi/180)):

                if(scanData[x] < closestObs):

                    closestObs = scanData[x]
                    ping_angle = x
                    
        elif(x > 90 - angleSwitch and x < 90 + angleSwitch):

            if(scanData[x] < BOX_HEIGHT/math.cos((x-90) * math.pi/180)):

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


def main():
    
    rospy.init_node('n')
    r = rospy.Rate(100)
    rospy.Subscriber("base_scan",LaserScan,laserCallback)    

    while not rospy.is_shutdown():
 
        straight()
    rospy.spin
if __name__ == '__main__':

    main()








    



    
