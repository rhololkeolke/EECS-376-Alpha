#Message that will be sent giving the status of segments issued by the Path Planner
#will be recieved by the Path Planner, the Look Ahead
#will be sent by the Velocity Node

#Requests a new path segment from the Path Planner Node
uint64 lastSegComplete

#path has been aborted and robot is stopped
bool abort

#incrementing segment number
uint64 seg_number

#Distance made (Progress made thus far)
float64 progress_made
