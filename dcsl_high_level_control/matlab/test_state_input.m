% testing state message input
clear all;
clc;

% add the ipc_bridge_matlab binaries to your path
%addpath('/opt/ros/fuerte/stacks/ipc-bridge-32bit-3.29.2011/ipc_msgs/ipc_geometry_msgs/bin');
[a, p] = system('rospack find ipc_geometry_msgs');
addpath(strcat(p, '/bin')); 

% create a publisher that publishes a geometry_msgs/Twist message
pid=geometry_msgs_PoseArray('connect','subscriber','state_module','state_message');

% create an empty geometry_msgs/Twist message structure
msg=geometry_msgs_PoseArray('empty');

% read a message and print to screen
while (1)
    msg = geometry_msgs_PoseArray('read',pid,100);
    if strcmp(class(msg),'struct')
        msg.linear.x
        msg.linear.y
        msg.linear.z
    end
end
