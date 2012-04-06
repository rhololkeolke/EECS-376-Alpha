#!/usr/bin/env python
import roslib; roslib.load_manifest('look_ahead')
import math
import rospy
from sensor_msgs.msg import LaserScan
from msg_alpha.msg._Obstacles import Obstacles


<<<<<<< HEAD
scanData=[]  #laser scan data
BOX_WIDTH = 0.5 #width for the bounded box if the path segment is straight(1)
BOX_HEIGHT = 1.0 #height for bounded box if the path segment is 1
angleSwitch = math.atanh(BOX_WIDTH) * 180/math.pi  #angle to switch distance formulas within the bounded box
ping_angle = None #the angle at which the lidar scanner picks up an obstacle

#Globals are only temporary for testing of logic if path segment is arc(2)
segType = 2  #segment type
curvature = None  #curvature of the path
segLenth = None  #length of the segment
initTanAngle = None  #inital tangent angle of the path segment
refPointX = None  #reference point
refPointY = None


#Receives path segment information from the path_seg topic
#@param Segment Status Data
#@type ROS msg ?
#@return nothing
def pathSegCallback(segData):
    global segType
    global curvature
    global segLenth
    global initTanAngle
    global refPointX
    global refPointY

    segType = segData.seg_type
    curvature  = segData.curvature
    segLength = segData.seg_length
    initTanAngle = segData.init_tan_angle
    refPointX = segData.ref_point.x
    refPointY = segData.ref_point.y

=======



scanData=[]
BOX_WIDTH = 0.45
BOX_HEIGHT = 1.2
angleSwitch = math.atanh(BOX_WIDTH) * 180/math.pi
ping_angle = None #the angle at which the lidar scanner picks up an obstacle

>>>>>>> develop
#Receives laser data from the base_scan topic and places the data into an array
#@param laserScan data
#@type tupile
#@return nothing
def laserCallback(data):
<<<<<<< HEAD
    global segType

=======
    #copy each element in the base_scan array to scanData[]
    #the i is always within the length of the base_scan array
>>>>>>> develop
    obsPub = rospy.Publisher('obstacles', Obstacles)     #Data should be published to the obstacles topics using the Obstacles message type                         
    obsData = Obstacles() #initalize an Obstacle message                     

    #find distance to closest objects on left and right and publish distances
    obsData.left_dist = data.ranges[0]   #left distance
    obsData.rt_dist = data.ranges[180]   #right dista 

    #redundant but necessary for compilation
    obsData.wall_dist_lt = data.ranges[0]   #left distance
    obsData.wall_dist_rt = data.ranges[180]   #right distance



    obsPub.publish(obsData)     
    
<<<<<<< HEAD
    #Run function to run based on the path segment type
    if(segType == 1):
        straight(data.ranges)
    elif(segType == 2):
        arc(data.ranges)
    elif(segType == 3):
        spin(data.ranges)
    elif(segType == None):
        print "Segment Type is Null. I don't know what to do"
    else:
        print "Invalid Segment Type passed into Look Ahead. I don't know what to do."
=======
    #obsData.wall_dist_left = data.ranges[0]
    #obsData.wall_dist_right = data.ranges[180]
    #print len(data.ranges)
    obsPub.publish(obsData)
    
    straight(data.ranges)
    #print len(scanData)
>>>>>>> develop

#Determine if there are obstacles along a straight path        
#@param laserScan data
#@type tupile
#@return nothing
def straight(scanData):
    obsPub = rospy.Publisher('obstacles', Obstacles)     #Data should be published to the obstacles topics using the Obstacles message type
    obsData = Obstacles() #initalize an Obstacle message
    closestObs = 90.0 # the range of the lidar scanner is 80m therefore no data should be beyond this value
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
<<<<<<< HEAD
        
    obsPub.publish(obsData)



#Determines if there is an obstacle within the path segment.
#@param data from the laser scan
#@type tupile
#@return nothing
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
    circles = [] #a list of each circle, each circle will be compared to the path circle.


    #The center of the path segment circle
    pathCentX = refPointX
    pathCentY = refPointY
    pathCircle = [refPointX,refPointY]
    
    #since curvature is 1/radius the radius should be 1/curvature
    #if the curvature is positive simply do the recpricoal
    if(curvature > 0):
        pathRadius = 1/curvature 

    #if the curvature is negative do the negative reciprocal
    elif(curvature < 0): 
        pathRadius = -1/curvature
    

    #make each lidar ping is the midpoint of a circle with a radius of 0.5
    #
        for i in range(len(scanData)):


            #Do not turn lidar pings within the path segment circle or along the path segment into circles                                                                        #works under the premise that all points are within the radius of the circle and therefore                                                                            #dx = xCenter - x;                                                                                                                                                    #dy = yCenter - y;                                                                                                                                                    #dx*dx + dy+dy < radius * radius                                                                                                                                      #j is always within range of the lidar pings                                                                                                               
            for j in range(len(scanData)):
                    
                dx = pathCentX - j
                dy = PathCentY- scanData(j)
                
                if(dx * dx + dy * dy < pathRadius * pathRadius):
                    
                    pass

                else:

                    centX = i
                    centY = scanData(i)
                    circle = [centX,centY]
                    circles.append(circle)
            
                        
        #if the euclidean distance between a circle and the radii of the path segment circle is less than the sum of the radii then an obstacle exists else it does not
        for c in ranges(circles):
            if(math.sqrt(math.pow(circles[c][0] - pathCircle[0],2.0) + math.pow(circles[c][1] - pathCircle[1],2.0))  < radius + pathRadius):
            
                obsData.exists = True   #an obstacle exists
            
            else:
                obsData.exists = False  #an obstacle does not exists
                
        obsPub.publish(obsData)  #publish the obstacle information

def spin(scanData):
    global BOX_WIDTH #width of the bounded box for checkAhead
    closestObs = 90.0 #distance for bounded box alg

    checkRight = False #scan to the right
    checkLeft = False #scan to the left
    checkAhead = False #scan ahead
    

    obsPub = rospy.Publisher('obstacles', Obstacles)     #Data should be published to the obstacles topics using the Obstacles message type                         
    obsData = Obstacles() #initalize an Obstacle message                     
=======

    obsData.wall_dist_right = scanData[0]
    obsData.wall_dist_left = scanData[180]

    obsPub.publish(obsData)
    #print len(scanData)
>>>>>>> develop

    #bounded box algorithim founded in straight()
    for x in range(len(scanData)):
    
<<<<<<< HEAD
        if(x < 90-angleSwitch or x > 90+angleSwitch):
            
            if(scanData[x] < BOX_WIDTH/math.cos((180.0-x) * math.pi/180.0)):
=======
    rospy.init_node('look_ahead')
    r = rospy.Rate(100)
    rospy.Subscriber("base_scan",LaserScan,laserCallback)    
    rospy.spin()
>>>>>>> develop

                if(scanData[x] < closestObs):
                    
                    closestObs = scanData[x]
                    ping_angle = x
                    
                elif(x > 90 - angleSwitch and x < 90 + angleSwitch):
                    
                    if(scanData[x] < BOX_HEIGHT/math.cos((x-90) * math.pi/180.0)):
                        
                        if(scanData[x] < closestObs):
                            
                            closestObs = scanData[x]
                            ping_angle = x
                            
    if(closestObs < 90):
        checkAhead = True
                                
    else:
        checkAhead = False


    #if an obstace is less than the box width on the left and right and some leeway then an obstacle exists
    if(scanData[0] < BOX_WIDTH + 0.2 ): #check left
        checkLeft = True
    if(scanData[180] < BOX_WIDTH + 0.2): #check right
        checkRight = True

    #if there is an obstacle on the Left, Right or infront then an obstalce exists otherwise, there is no obstacle
    if(checkAhead == True or checkRight == True or checkLeft == True):
        obsData.exists = True
    else:
        obsData.exists = False

        obsPub.publish(obsData) #publish the data


#the subscriber calls the laserCallback which determines which function to run based on the segType found in the segCallback
def main():
    rospy.init_node('n')  #initialize node with the name n
    rospy.Subscriber("base_scan",LaserScan,laserCallback) #node subscribes to base_scan topic which is of type LaserScan which invokes the laserCallback with the msg as first arg
    rospy.spin()       
    
if __name__ == '__main__':

    main()

