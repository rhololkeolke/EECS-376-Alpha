#The message tells whether or not an obstacle exists on current and next path segements and the distance along the path

#Used to determine whether or not an obstacle exists
bool exists
uint64 distance

#Distance to the closest obstacle on the left and on the right
uint64 left_dist
uint64 rt_dist

#redundant but necessary for compilation in the branch
uint64 wall_dist_rt
uint64 wall_dist_lt



#Tells the angle at which the closest ping is, should there be one.
uint64 ping_angle

