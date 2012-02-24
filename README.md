# EECS 376/476 Team Alpha Code

Devin Schwab    dts34
Eddie Massey    eem33
Mark Schultz    mxs802
David Jannotta  daj40

The latest version of our code can be found at http://www.github.com/rhololkeolke/EECS-376-Alpha

## Problem-Set-4

Currently velocity profiler has hard coded values for path that take the robot from the elevators to the vending machine. 

This is based on the code from Problem Set 2 and 3 which is based on the example Matlab code.  However, there are some additions and changes.

Instead of a Simu function like the Matlab code we created a class called State.  This class is responsible for keeping track of where the velocity profiler node thinks its at.  This can be extended in the future to take in actual data from the lidar and other sensors.

The original Matlab code only allowed positive values for the function.  Our function straight takes in a distance.  If distance is positive the robot will move forward that distance.  Otherwise it will move backwards the absolute value of that distance.  The function angle takes in an angle in radians.  If angle is negative then the robot will spin clockwise along its z axis and if the angle is positive then the robot will spin counterclockwise along its z axis.

To incorporate the eStop into our code.  We created a callback that updates a global variable called stopped.  When stopped is true the eStop is engaged and when it is false it is not.  To make sure that stopped is updated whenever a new value of motors_enabled is published we are running an instance of the AsyncSpinner class.  This class runs each callback on a nonblocking thread.  Which means that stopped is always being updated even when velocity profiler isn't looking at it.

When stopped is enabled the instance of the state is updated so that the robot knows it is not moving.  Also the velocity and omega commands are set to 0.  We had trouble with the robot not responded to velocity commands shortly after the eStop was disengaged.  To solve this we simply added a 2.0 second sleep timer that runs when the robot is reenabled.  We also made sure to send out 0.0 velocity commands.  This is not necessary for the actual robot as the eStop disengages power to the motor. However, to test the eStop response in the simulator this was needed.  We simply used a publisher class, publishing to the motors_enabled topic to simulate eStop presses.

We will be updating velocity profiler to work with messages from a Path Planner node that are of the form PathSegment.msg (from blackboard).
