function  send_trajectory(robot, velmsg, vd)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

% Send desired velocities to the robot Linear
vx = vd(1);
vy = vd(2);
vz = vd(3);


velmsg.Pose.Position.X = vx;
velmsg.Pose.Position.Y = vy;
velmsg.Pose.Position.Z = vz;



send(robot,velmsg);
end
