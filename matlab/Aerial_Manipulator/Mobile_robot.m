% Matlab Conection %

%% CLEAN VARIABLES
clc,clear all,close all;

%% DEFINITION OF TIME VARIABLES
ts = 0.05;
tf = 30;
to = 0;
t = (to:ts:tf);

%% ROS PARAMETER FOR COMUNICATION
rosshutdown
active = true;
Master = 'http://190.124.230.188:11311';
Local = '190.124.230.188';
ROS_Options(Master,Local,active);
%rosinit('192.168.0.104', 'NodeHost', '192.168.0.105', 'Nodename', '/Matlab_Communication');

%% OBJECTS CREATION OF TOPICS ROS
robot = rospublisher('/mobile/cmd_vel');
velmsg = rosmessage(robot);
odom = rossubscriber('/mobile/odom');

%% SET VALUES TO ZERO
send_velocities(robot, velmsg, [0, 0, 0, 0, 0 , 0])
%% SYSTEM GEOMETRIC PARAMETERS
a = 0.2;

%% GENERAL VECTOR PARAMETERS
L1 = [a];

%% READ VALUES FOR INITIAL CONDITIONS
[q(:, 1), qp(:, 1)] = odometry(odom, L1);

%% VECTOR OF CONTROL VELOCITIES
u = [0.0*ones(1, length(t));...
     0.0*ones(1, length(t))];
   
%% DESIRED TRAJECTORY
[xd,yd,zd,psid,xdp,ydp,zdp,psidp] = Trajectory(t, ts, 3);   

qd = [xd;...
      yd];
  
qdp = [xdp;...
       ydp];
 
%% CONTROL
K1 = 1*eye(2);
K2 = 1*eye(2);

%% CONTROL SECTION
chi = [0.3037;0.2768;-0.0004018;0.9835;0.003818;1.0725];
mobile_1 = mobile_robot_dynamic(L1, chi, q(:,1), ts);
controller = controller_robot(K1, K2, mobile_1);

for k=1:1:length(t)
    tic; 
    %% GENERAL VECTOR OF ERROR SYSTEM
    qe(:, k) = qd(:,k)-q(1:2,k);
    
    u(:, k) = controller.kinematic_controller(qd(:, k), qdp(:, k), q(:, k));
    
    % Derivative Desired
    if k > 1
        up(: ,k) = (u(:, k) - u(:, k-1))/ts;
    else
        up(:, k) = [0; 0];
    end
    
    u_ref(:, k ) = controller.Dynamic_controller(u(:, k), up(:, k), q(:, k), qp(:, k));
   
    %% SEND VALUES OF CONTROL ROBOT
    send_velocities(robot, velmsg, [u_ref(1, k), 0, 0, 0, 0 , u_ref(2, k)]);
    
    %% GET VALUES OF MOBILE
    [q(:, k+1), qp(:, k+1)] = odometry(odom, L1);
    
    while(toc<ts)
    end
    toc;
end
%% SET VALUES TO ZERO ON THE DESIRED VELOCITIES
send_velocities(robot, velmsg, [0, 0, 0, 0, 0 , 0])
largo = 0.4;
ancho = 0.3;
SIMULACION(a,largo,ancho,q(1,:),q(2,:),qd(1, :),qd(2, :),q(3, :),ts);
% Save Data
save("Mobile_Kinematics_Ros.mat", "t", "q", "u", "qd", "qe", "ts", "qp");
