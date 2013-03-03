ROS Platform Code for DCSL at Princeton University

Description:

This code is designed to run the Miabot multi-agent system at DCSL at Princeton Unversity. The system is a closed loop control system with differential drive robots commanded over bluetooth. The poses of the robots are sensed with an overhead camera. Waypoint control for the robots or velocity and angular velocity control can be controlled from MATLAB.

************************

Dependencies:

This system requires the fuerte version of the  ROS system. Installation instructions can be found at http://www.ros.org/wiki/fuerte/Installation.

It is also dependent on: 
ROS packages
 -turtlebot-apps for joystick control.
 -camera1394 for image capture
External libraries
 -eigen for c++
 -munkres for Hungarian algorith in python
 -OpenCV for python for image processing

************************

Compiling:
Each package can be compiled using the following command from any directory.

rosmake <package name>

The dcsl_miabot_main package is dependent on all the packages necessary to run the DCSL Miabot platform. Therefore,

rosmake dcsl_miabot_main

will compile all packages necessary to run the system.

************************

Running:
To connect Miabots to the computer:
1. Turn Miabots on
2. Command: hcitool scan
   This returns the hardware addresses of the robots.
3. Command: sudo rfcomm connect <serial port> <hardware address>
   Run this for each robot. <serial port> should be rfcomm0, rfcomm1, etc. <hardware address> was found with hcitool above.
To run miabot_follower demo:
4. In new terminal command: roscore
5. In new terminal command: roslaunch dcsl_miabot_main miabot_follower.launch
   If miabot driver nodes fail to connect, disconnect robots from rfcomm and reconnect (step 3 above).
To shutdown system:
6. In all open tabs command ctrl+c to close process to the order of miabot_follower, roscore, rfcomm connections.
