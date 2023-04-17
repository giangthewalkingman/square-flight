# square-flight
In this package, drone can follow a square trajectory and land at original position.

Branches:

master: position control

vel-control: velocity control (adding velocity)

In terminal:

T1: 

roslaunch px4 mavros_posix_sitl.launch

T2, T3:
 
cd ~/catkin_ws

source ./devel/setup.bash

T2:
 
roslaunch square_flight offboard.launch

T3: 

rosrun square_flight setmode_offb
