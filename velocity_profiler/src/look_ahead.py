#!/usr/bin/env python
import roslib; roslib.load_manifest('laserScanner')
import math
import rospy
from sensor_msgs.msg import LaserScan

scanData=[]
exists = False


BOX_WIDTH = 0.5
BOX_LENGTH = 1
angleSwitch = math.atanh(BOX_WIDTH)
distance = 0

def laserCallback(data):
    for i in range(len(data.ranges)):
        scanData.append(data.ranges[i])

def straight():
    closestObs = 90
    for x in range(len(scanData)):
        
        if(x < 90 - angleSwitch or x > 90 + angleSwitch):
            if(scanData[x] < BOX_WIDTH/math.cos(180-x) * math.pi/180):
                if(scanData[x] < closestObs):
                    closestObs = scanData[x]

            elif(x > 90 - angleSwitch and x < 90 + angleSwitch):
                if(scanData[x] < BOX_LENGTH/math.cos(x-90) * math.pi/180):
                    if(scanData[x] < closestObs):
                        closestObs = scanData[x]
    if(closestObs < 90):
        exists = true
        distance = closestObs

def main():
    rospy.init_node('n')
    rospy.Subscriber("base_scan",LaserScan,laserCallback)
#    obsPub = rospy.Publisher('obstacles',1)
    r = rospy.Rate(10)

    while not rospy.is_shutdown():
        straight()
        rospy.spin()
        r.sleep()
        

if __name__ == '__main__':
    main()








                    



                        
