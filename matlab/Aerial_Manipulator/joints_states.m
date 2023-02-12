function [q,qp] = joints_states(odom, L)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

% Robot constant parameters
a = L(1);


% Read joints RoS
joints_data = receive(odom,3);
q = joints_data.Position;
qp = joints_data.Velocity;

end