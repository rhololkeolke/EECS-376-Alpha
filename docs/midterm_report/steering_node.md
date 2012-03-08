## Steering Node ##
The Steering node is responsible for taking the desired velocities from the 
velocity planner, the desired path from the path planner, and the current 
position to determine correction factors to the velocity command such that the 
robot does not stray from its desired path.

### Theory of Operation ###
#### Input/Output ####
Listener 			Information
-------- 			-----------
command_velocity 	Desired speed
path_planner 		Desired path
odom 				Current location

Table: Input

Publisher 	Information
--------- 	-----------
cmd_vel 	Velocity vector

Table: Output

#### Steering Algorithm ####
We will use the linear steering algorithm. This algorithm takes a number of 
inputs such as:

* The $x$ and $y$ coordinates of the destination.
* The current $x$ and $y$ coordinates.
* Tuning parameters $K_d$ and $K_\theta$

First we use the path segment coordinates to calculate the desired heading with 
the following: $atan(yf-ys,xf-xs)$. Next we find a $d_\theta$ by subtracting 
$heading_{dest}-heading_{curr}$. We can prevent turning the long way by 
checking to see that $d_\theta$ is less than or greater than $\pi$. Finally, we 
take the vector components of the desired heading $tx=cos(heading_{dest})$, 
$ty=-sin(heading_{dest})$ and dot these with the vectors from the start point 
to the current point $xrs*nx+yrs*ny$. We take this product and add it to 
$d_\theta$ to get the final corrected heading 
$-K_d*offset+K_{\theta}*d_\theta$.

### Observations ###

### Coding Procedure ###
The `steering_example.cpp` sample code was tweaked to include a file
reader to allow us to easily modify constants $K_d$ and $K_\theta$.
