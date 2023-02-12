function  send_joints_velocities(robot, msg, qpd)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

% Send desired velocities to the robot Linear
q1pd = qpd(1);
q2pd = qpd(2);
q3pd = qpd(3);

msg.Data = [q1pd, q2pd, q3pd];
send(robot,msg);
end

