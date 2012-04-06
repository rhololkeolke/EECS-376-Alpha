#!/usr/bin/env python
import roslib; roslib.load_manifest('look_ahead')
import math
import rospy
from sensor_msgs.msg import LaserScan
from msg_alpha.msg._Obstacles import Obstacles


scanData=[]  #laser scan data
BOX_WIDTH = 0.5 #width for the bounded box if the path segment is straight(1)
BOX_HEIGHT = 1.0 #height for bounded box if the path segment is 1
angleSwitch = math.atanh(BOX_WIDTH) * 180/math.pi  #angle to switch distance formulas within the bounded box
ping_angle = None #the angle at which the lidar scanner picks up an obstacle

#Globals are only temporary for testing of logic if path segment is arc(2)
segType = None  #segment type
curvature = None  #curvature of the path
segLenth = None  #length of the segment
initTanAngle = None  #inital tangent angle of the path segment
refPoint = None  #reference point


#Receives path segment information from the path_seg topic
#@param Segment Status Data
#@type ROS msg ?
#@return nothing
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

#Receives laser data from the base_scan topic and places the data into an array
#@param laserScan data
#@type tupile
#@return nothing
def laserCallback(data):
    global segType

    obsPub = rospy.Publisher('obstacles', Obstacles)     #Data should be published to the obstacles topics using the Obstacles message type                         
    obsData = Obstacles() #initalize an Obstacle message                     

    #find distance to closest objects on left and right and publish distances
    obsData.left_dist = data.ranges[0]   #left distance
    obsData.rt_dist = data.ranges[180]   #right dista 

    #redundant but necessary for compilation
    obsData.wall_dist_lt = data.ranges[0]   #left distance
    obsData.wall_dist_rt = data.ranges[180]   #right distance



    obsPub.publish(obsData)     
    
    #Run function to run based on the path segment type
    if(segType == 1):
        straight(data.ranges)
    elif(segType == 2):
        arc(data.ranges)
    elif(segType == 3):
        spin(data.ranges)
    elif(segType == None):
        print "Segment Type is Null therefore I will simply check for obstacles as if it were an arc"
    else:
        print "Invalid Segment Type passed into Look Ahead"

#Determine if there are obstacles along a straight path        
#@param laserScan data
#@type tupile
#@return nothing
def straight(scanData):
    obsPub = rospy.Publisher('obstacles', Obstacles)     #Data should be published to the obstacles topics using the Obstacles message type
    obsData = Obstacles() #initalize an Obstacle message
    closestObs = 90.0 # the range of the lidar scanner is 80m therefore no data should be beyond this value
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





def spin(scanData):
    global BOX_WIDTH #width of the bounded box for checkAhead
    closestObs = 90.0 #distance for bounded box alg

    checkRight = False #scan to the right
    checkLeft = False #scan to the left
    checkAhead = False #scan ahead
    

    obsPub = rospy.Publisher('obstacles', Obstacles)     #Data should be published to the obstacles topics using the Obstacles message type                         
    obsData = Obstacles() #initalize an Obstacle message                     

    #bounded box algorithim founded in straight()
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








    



    
