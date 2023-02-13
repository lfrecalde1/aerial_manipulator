function [q,qp] = joints_states(odom)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

% Read joints RoS
joints_data = receive(odom,3);
q = joints_data.Position;
qp = joints_data.Velocity;

end