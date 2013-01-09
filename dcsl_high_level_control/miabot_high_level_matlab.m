% miabot_high_level_matlab.m
% script for subscribing to state and publishing either waypoint or velocity control
% Will Scott 2013
clear all;

% add the ipc_bridge_matlab binaries to your path
% for some reason rospack is not working here (possibly matlab doesn't look at .bashrc or something)
% so just put the absolute path to the ipc-bridge install directory for now
%[a, p] = system('rospack find ipc_geometry_msgs');
%addpath(strcat(p, '/bin')); 
addpath('/home/illscott/ROS_workspace/ipc-bridge-32bit-3.29.2011/ipc_msgs/ipc_geometry_msgs/bin');

%% create a subscriber that subscribes to the state estimate
pid_state = geometry_msgs_PoseArray('connect','subscriber','state_module','state_message')

%% create publishers that publish waypoints and velocities
pid_way = geometry_msgs_PoseArray('connect','publisher','waypoint_module','waypoint_message')
pid_vel = geometry_msgs_PoseArray('connect','publisher','velocity_module','velocity_message')

%% create an empty geometry_msgs/poseArray message structure
state_msg = geometry_msgs_PoseArray('empty');
way_msg = geometry_msgs_PoseArray('empty');
vel_msg = geometry_msgs_PoseArray('empty');

% create an empty geometry_msgs/Pose to put into poseArrays later
pose = geometry_msgs_Pose('empty');

%% THE BIG LOOP WHERE EVERYTHING HAPPENS
% create this stoploop button to allow the user to stop the program
% without pressing CTRL-C, allowing us to close connections gracefully
FS = stoploop('Press this to stop control loop') ;
while ~FS.Stop()
	% try to read the latest state estimate from its topic
	state_msg = geometry_msgs_PoseArray('read',pid_state,1);
	% if the read was unsuccessful, state_msg will be a 1x0 cell
	while ~strcmp(class(state_msg),'struct')
	    state_msg = geometry_msgs_PoseArray('read',pid_state,1);
	    pause(0.01);
	end

	% create and populate a matrix to hold states,
	% one row for each robot, with entries [x, y, theta]
	num_robots = length(state_msg.poses);
	state_mat = zeros(num_robots,3);
	for k = 1:num_robots
		state_mat(k,1) = state_msg.poses{k}.position.x;
		state_mat(k,2) = state_msg.poses{k}.position.y;
		state_mat(k,3) = state_msg.poses{k}.orientation.z;
	end

	% call the user-defined control law function
	[way_mat vel_mat] = miabot_control_law(state_mat);

	% publish waypoint if waypoints are given,
	% otherwise publish velocities
	if ~isempty(way_mat)
		% populate the way_msg
		for k=1:num_robots
			pose.position.x = way_mat(k,1);
			pose.position.y = way_mat(k,2);
			way_msg.poses{k} = pose;
		end
		% publish the message
		geometry_msgs_PoseArray('send',pid_way,way_msg);
	else
		% populate the vel_msg
		for k=1:num_robots
			pose.position.x = vel_mat(k,1);
			pose.orientation.z = vel_mat(k,2);
			vel_msg.poses{k} = pose;
		end
		% publish the message
		geometry_msgs_PoseArray('send',pid_vel,vel_msg);
	end

end % end of big control loop

% make sure velocities of robots are set to zero
pose.position.x = 0;
pose.orientation.z = 0;
for k=1:num_robots
	vel_msg.poses{k} = pose;
end

% finally disconnect the ipc connections
geometry_msgs_PoseArray('diconnect',pid_state);
geometry_msgs_PoseArray('diconnect',pid_way);
geometry_msgs_PoseArray('diconnect',pid_vel);