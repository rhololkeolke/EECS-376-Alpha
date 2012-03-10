# Steering Node #
The Steering node is responsible for taking the desired velocities from the 
velocity planner, the desired path from the path planner, and the current 
position to determine correction factors to the velocity command such that the 
robot does not stray from its desired path.

## Theory of Operation ##
describe what we want the steering node to do in more exact detail. Say how 
they interconnect and how we use the messages
We use a number of messages to control the steering node.
* Path segment: the path segment node passes in the current path segment we 
  would like the steering node to follow.
* vel_des: this message gives us the desired velocity as determined by the 
  velocity_profiler.
* TODO: others?

### Steering Algorithm ###
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

## Observations ##
We ran into several problems when trying to perfect our steering code. The 
hardest part of this demo was getting all of the previous nodes integrated and 
functioning. We had to integrate several dummy messages and structures to glue 
together everything until all the nodes can be completed.

-todo: describe bug where the robot would get too close to the door on the 
first turn

## Coding Procedure ##
The `steering_example.cpp` sample code was tweaked to include a file
reader to allow us to easily modify constants $K_d$ and $K_\theta$. We also 
ended up coding a rudimentary path planner that has hard coded path segments. 

## Future Plans ##
* Non-linear steering:
  We plan on replacing the linear steering algorithm with the non-linear one
* Arc path steering:
  We plan on generating arch path segments and allowing the steering node to 
  maintain control over these segments.
* Python source code:
  We plan on porting all of our existing code to python. We would like to 
  utilize rospy for ease of programming and to get rid of compile time.
