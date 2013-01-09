function [way_mat vel_mat] = miabot_control_law(state_mat)
% miabot_control_law.m, an example control law for miabots
% demonstrating kuramoto coupled oscillators
%
% input: state_mat - with number of rows equal to number of robots
%                    columns correspond to [x, y, theta] states
% outputs: way_mat - matrix of desired waypoints, with columns
%                    corresponding to [x, y] of desired position
%          vel_mat - matrix of desired velocities, with one row
%                    for each robot, linear velocity in the first
%                    column and angular velocity in second
%
% note: choose to output either waypoints or velocities, and
%       return an empty matrix [] for the one not used.
%
% kuramoto control law: 
%   theta_dot(i) = omega(i) + (k/n)*sum of sin(theta(j)-theta(i)) for i~=j
	omega = 0.2;
	k = 1;
	num_robots = size(state_mat, 1);
	theta = state_mat(:,3);
	theta_dot = omega*ones(num_robots,1);


	
	for m = 1:num_robots
		for n = 1:num_robots
			if m ~= n
				theta_dot(m) = theta_dot(m) + ...
					(k/num_robots)*sin(theta(n)-theta(m));
		end
	end

	way_mat = [];
	vel_mat = zeros(num_robots, 2);
	vel_mat(:,2) = theta_dot
end