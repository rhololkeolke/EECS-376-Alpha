#The message tells whether or not an obstacle exists on current and next path segements and the distance along the path

#Used to determine whether or not an obstacle exists
bool exists
float64 distance

#Distance to the closest obstacle on the left and on the right
float64 left_dist
float64 right_dist

#Distance to the closest objects at 0 and 181
float64 wall_dist_left
float64 wall_dist_right

#Tells the angle at which the closest ping is, should there be one.
uint64 ping_angle

