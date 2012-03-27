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

#Globals are only temporary for testing of logic
segType = None
curvature = None
segLenth = None
initTanAngle = None
refPoint = None


#Receives path segment information from the path_seg topic
def pathSegCallback(segData):
    global segType
    global curvature
    global segLenth
    global initTanAngle
    global refPoint

    segType = segData.seg_type
    curvature  = segData.curvature
    segLength = segData.seg_length
    initTanAngle = segData.init_tan_angle
    refPoint = segData.ref_point
    
    #segInfo = [segType,curvature,segLength,initTanAngle,refPoint]
#    return segInfo

#Receives laser data from the base_scan topic and places the data into an array
def laserCallback(data):
    obsPub = rospy.Publisher('obstacles', Obstacles)     #Data should be published to the obstacles topics using the Obstacles message type                         
    obsData = Obstacles() #initalize an Obstacle message                     
    
    obsData.wall_dist_left = data[0]
    obsData.wall_dist_rt = data[181]
    obsPub.publish(obsData)
    
    straight(data.ranges)
    arc(data.ranges)
    #print len(scanData)

#Determine if there are obstacles along a straight path        
def straight(scanData):
    obsPub = rospy.Publisher('obstacles', Obstacles)     #Data should be published to the obstacles topics using the Obstacles message type
    obsData = Obstacles() #initalize an Obstacle message
    closestObs = 90 # the range of the lidar scanner is 80m therefore no data should be beyond this value
    global ping_angle #refernce to the global


    #Check for obstacles to left and to the rignt
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
    print len(scanData)



#Determines if there is an obstacle within the path segment.
#@param data from the laser scan
#@type tupile
#@param path segment info
#@type list
#@return Obstacle existance and distance
def arc(scanData):
    obsPub = rospy.Publisher('obstacles', Obstacles)     #Data should be published to the obstacles topics using the Obstacles message type
    obsData = Obstacles() #initalize an Obstacle message

    #    seg = pathSegCallback(segInfo) #call the path seg callback 

    #globals here temporarily for testing
    global curvature
    global segLenth
    global initTanAngle
    global refPoint

    radius = 0.5 #radius of the circle to be made around each lidar ping
    theta = 0.0 #angle measure of circle
    circles = []

    #The starting point for the path segment of the robot should be (0,0) assuming the paths are passed in for scanning at the robots present location
    xStartPt = 0 
    yStartPt = 0

    #HOPEFULLY the end point will be generated from a message
    xEndPt = None
    yEndPt = None 


    #Make a circle around the path segment
    #Make a circle around each ping excluding the path segment
    #If the distance between the circles is less than the sum of the radii an obstacle exists
    #invariant: the iterator is always within bounds of the 181 pings of lidar data
    for i in ranges(scanData):
        
        #if the lidar ping has reached the end point of the projected path segment
        #create a circle around the path segment
        if(i == xEndPt and scanData(i) == yEndPt): 
            pathRadius = math.tan(i)/(math.sqrt(math.pow(i - xStartPt),2.0) + math.pow(scanData(i) - yStartPt)) #tangent of the angle divded by the euclidean distance give the radius of the arc as if it were a circle
            xPathCent = i * pathRadius * math.sin(i)
            yPathCent = scanData(i) * pathRadius * math.sin(i)
            pathCircle = [xPathCent,yPathCent]
            

        #create an circle around each lidar ping with the ping as the center
        #??? Must figure out mathematically how to exclude the path segment circle form these calculations
        while(theta < 2 * math.pi):
            
            
            #Do not turn lidar pings within the path segment circle or along the path segment into circles
            #works under the premise that all points are within the radius of the circle and therefore
            #dx = xCenter - x;
            #dy = yCenter - y;
            #dx*dx + dy+dy < radius * radius
            #j is always within range of the lidar pings
            for j in ranges(scanData):
                insideCircle = False

                dx = xPathCent - j ;
                dy = yPathCent- scanData(j);
                
                if(dx * dx + dy * dy < pathRadius * pathRadius):
                
                
                else:
                    
                theta += 0.1
                xCenter = i * radius * math.sin(theta) #create the x-coord for the center of the circle
                yCenter = scanData(i) * radius * math.cos(theta) # create the y-coord for the center of the circle
                circle = [xCenter,yCenter] #a list of circle information contains ifs centerpoint and location along lidar ping
                circles.append(circle)  # place each circle in a list of circles

        #if the euclidean distance between a circle and the radii of the path segment circle is less than the sum of the radii then an obstacle exists else it does not
        for c in ranges(circles):
            if(math.sqrt(math.pow(circles[c][0] - pathCircle[0],2.0) + math.pow(circles[c][1] - pathCircle[1],2.0)))) -   < radius + pathRadius))
            
                obsData.exists = True   #an obstacle exists
            
            elif:
                obsData.exists = False  #an obstacle does not exists
                
        obsPub.publish(obsData)  #publish the obstacle information
        

        

        #publish that an obstacle exists


def main():
    
    rospy.init_node('n')
    r = rospy.Rate(100)
    rospy.Subscriber("base_scan",LaserScan,laserCallback)    
    rospy.spin()

if __name__ == '__main__':

    main()








    



    
