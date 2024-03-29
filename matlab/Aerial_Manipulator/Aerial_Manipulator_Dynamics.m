%% CLEAN VARIABLES
clc, clear all, close all;

%% LOAD DATA 
load('Ident_real_1_1.mat');

%% TIME
t;
dt_time = dt;
ts;
N = length(t);

%% POSE OF THE SYSTEM THESE VALUES WERE OBTAINED RESPECT TO THE INERTIAL FRAME
%% POSITION DRONE
x = xu;
y = yu;
z = zu;
%% ORIENTATION JUST YAW VALUE DRONE
yaw = psi;
%% MANIPULATOR
q1;
q2;
q3;

%% VELOCITIES IN THE INERTIAL FRAME
xp = xu_p;
yp = yu_p;
zp = zu_p;

%% ANGULAR VELOCITIES 
wz = psi_p;

%% ROBOT ARM
q1p = q1_p;
q2p = q2_p;
q3p = q3_p;

%% GENERAL VECTOR DEFINITION
h = [x;...
     y;...
     z;...
     yaw];
hp = [xp;...
      yp;...
      zp;...
      wz];
q = [q1;...
     q2;...
     q3];
q3p = Filtro(q3p, t);
qp = [q1p;...
      q2p;...
      q3p];
%% DUE THE SYSTEM USES INERTIAL COORDINATE SYSTEM WE NEED TO TRANSFORM THESE INFORMATION TO THE BODY FRAME
v = frame_transformation(h, hp, t);

ul = v(1, :);
um = v(2, :);
un = v(3, :);
w = v(4, :);

%% REFERENCE SIGNALS OF THE DRONE
ul_ref;
um_ref;
un_ref;
w_ref;

%% REFERENCE MANIPULATOR
q1p_ref;
q2p_ref;
q3p_ref;

%% GENERAL VECTOR OF THE SIGNALS DRONE
%% VELOCITIES OF THE SYSTEM
v;

%% REFERENCE OF THE SYSTEM
vref = [ul_ref;...
        um_ref;...
        un_ref;...
        w_ref];
    
%% ACCELERATION OF THE SYSTEM
ulp = [0 , diff(ul)/ts];
ump = [0 , diff(um)/ts];
unp = [0 , diff(un)/ts];
wp = [0 , diff(w)/ts];

vp = [ulp; ump; unp; wp];

%% GENERAL VELOCITIES OF THE MANIPULATOR
qp;

%% REFERENCE OF THE SYSTEM
qp_ref = [q1p_ref; q2p_ref; q3p_ref];

q1pp =  [0 , diff(q1p)/ts];
q2pp =  [0 , diff(q2p)/ts];
q3pp =  [0 , diff(q3p)/ts];

qpp = [q1pp; q2pp; q3pp];

%% PHYSICAL PARAMETERS
L1 = [0];

%% ROS PARAMETER FOR COMUNICATION
rosshutdown
active = true;
Master = 'http://192.168.88.244:11311';
Local = '192.168.88.244';
ROS_Options(Master,Local,active);

%% OBJECTS CREATION OF TOPICS ROS
robot_cmd = rospublisher('/aerial_manipulator/cmd_vel');
cmd_msg = rosmessage(robot_cmd);

robot_joints = rospublisher('/aerial_manipulator/joints_ref');
joint_msg = rosmessage(robot_joints);

odom = rossubscriber('/aerial_manipulator/odom');
joints = rossubscriber('/aerial_manipulator/joints');

%% READ INITIAL VALUES
[h_drone(:, 1), hp_drone(:, 1)] = odometry(odom);
[q_drone(:,1), qp_drone(:,1)] = joints_states(joints);

%% SET VALUES TO ZERO
send_velocities(robot_cmd, cmd_msg, [0, 0, 0, 0, 0 , 0]);
send_joints_velocities(robot_joints, joint_msg, [0, 0, 0]);


for k=1:1:length(t)-1
    tic; 
    %% GENERAL VECTOR OF ERROR SYSTEM
    
   
    %% SEND VALUES OF CONTROL ROBOT
    send_velocities(robot_cmd, cmd_msg, [vref(1, k), vref(2, k), vref(3, k), 0, 0 , vref(4, k)]);
    send_joints_velocities(robot_joints, joint_msg, [qp_ref(1, k), qp_ref(2, k), qp_ref(3, k)]);

    %% GET VALUES OF MOBILE
    [h_drone(:, k+1), hp_drone(:, k+1)] = odometry(odom);
    [q_drone(:, k+1), qp_drone(:, k+1)] = joints_states(joints);
    while(toc<ts)
    end
    toc;
end

send_velocities(robot_cmd, cmd_msg, [0, 0, 0, 0, 0 , 0]);
send_joints_velocities(robot_joints, joint_msg, [0, 0, 0]);
%% RESULTS PLOTS
lw = 1; 
lwV = 2; 
fontsizeLabel = 11; 
fontsizeLegend = 11;
fontsizeTicks = 11;
fontsizeTitel = 11;
sizeX = 1300; 
sizeY = 750; 

%% COLOR PROPERTIES
c1 = [80, 81, 79]/255;
c2 = [244, 213, 141]/255;
c3 = [242, 95, 92]/255;
c4 = [112, 141, 129]/255;

C18 = [0 0 0];
c5 = [130, 37, 37]/255;
c6 = [205, 167, 37]/255;
c7 = [81, 115, 180]/255;

C1 = [246 170 141]/255;
C2 = [51 187 238]/255;
C3 = [0 153 136]/255;
C4 = [238 119 51]/255;
C5 = [204 51 17]/255;
C6 = [238 51 119]/255;
C7 = [187 187 187]/255;
C8 = [80 80 80]/255;
C9 = [140 140 140]/255;
C10 = [0 128 255]/255;
C11 = [234 52 89]/255;
C12 = [39 124 252]/255;
C13 = [40 122 125]/255;
%C14 = [86 215 219]/255;
C14 = [252 94 158]/255;
C15 = [244 171 39]/255;
C16 = [100 121 162]/255;
C17 = [255 0 0]/255;

%% PLOT SECTION
figure('Position', [500 500 sizeX sizeY])
set(gcf, 'Position', [500 500 sizeX sizeY]);
fig1_comps.fig = gcf;

%% Figure 1 Ul
axes('Position',[0.04 0.79 .45 .17]);
%% FRONTAL VELOCITY
ul = line(t,v(1, :));
set(ul, 'LineStyle', '-', 'Color', C11, 'LineWidth', 1.1*lw);
ul_m = line(t,hp_drone(1,:));
set(ul_m, 'LineStyle', '--', 'Color', C12, 'LineWidth', 1.1*lw);
ul_ref = line(t,vref(1,:));
set(ul_ref, 'LineStyle', '-.', 'Color', C9, 'LineWidth', 1.3*lw);


%% Title of the image
hTitle_1 = title({'$\textrm{(a)}$'},'fontsize',12,'interpreter','latex','Color',C18);
%xlabel('$\textrm{Time}[s]$','fontsize',10,'interpreter','latex','Color',C18);
ylabel('$[m/s]$','fontsize',9,'interpreter','latex', 'Color',C18);

%% Legend nomeclature
hLegend_1 = legend([ul,ul_m,ul_ref],{'$\mu_{l}$','$\mu_{lm}$','$\mu_{lref}$'},'fontsize',11,'interpreter','latex','Color',[255 255 255]/255,'NumColumns',1,'TextColor','black');
set(gca,'ticklabelinterpreter','latex',...
         'fontsize',1.1*fontsizeTicks)
     
%% Figure properties
ax_1 = gca;
ax_1.Box = 'on';
ax_1.BoxStyle = 'full';
ax_1.TickLength = [0.01;0.01];
ax_1.TickDirMode = 'auto';
ax_1.XTickLabel = [];
ax_1.YMinorTick = 'on';
ax_1.XMinorTick = 'on';
ax_1.XMinorGrid = 'on';
ax_1.YMinorGrid = 'on';
ax_1.MinorGridAlpha = 0.15;
ax_1.LineWidth = 0.8;
ax_1.XLim = [0 t(end)];

%% Figure 2 UM
axes('Position',[0.04 0.58 .45 .17]);
%% FRONTAL VELOCITY
um = line(t, v(2,:));
set(um, 'LineStyle', '-', 'Color', C3, 'LineWidth', 1.1*lw);
um_m = line(t, hp_drone(2,:));
set(um_m, 'LineStyle', '--', 'Color', C14, 'LineWidth', 1.1*lw);
um_ref = line(t, vref(2,:));
set(um_ref, 'LineStyle', '-.', 'Color', C9, 'LineWidth', 1.3*lw);


%% Title of the image
%xlabel('$\textrm{Time}[s]$','fontsize',10,'interpreter','latex','Color',C18);
ylabel('$[m/s]$','fontsize',9,'interpreter','latex', 'Color',C18);

%% Legend nomeclature
hLegend_2 = legend([um, um_m, um_ref],{'$\mu_{m}$','$\mu_{mm}$','$\mu_{mref}$'},'fontsize',11,'interpreter','latex','Color',[255 255 255]/255,'NumColumns',1,'TextColor','black');
set(gca,'ticklabelinterpreter','latex',...
    'fontsize',1.1*fontsizeTicks)
%% Figure properties
ax_2 = gca;
ax_2.Box = 'on';
ax_2.BoxStyle = 'full';
ax_2.TickLength = [0.01;0.01];
ax_2.TickDirMode = 'auto';
ax_2.XTickLabel = [];
ax_2.YMinorTick = 'on';
ax_2.XMinorTick = 'on';
ax_2.XMinorGrid = 'on';
ax_2.YMinorGrid = 'on';
ax_2.MinorGridAlpha = 0.15;
ax_2.LineWidth = 0.8;
ax_2.XLim = [0 t(end)];
% 
% 
%% Figure 3 UN
axes('Position',[0.04 0.37 .45 .17]);
%% FRONTAL VELOCITY
un = line(t, v(3, :));
set(un, 'LineStyle', '-', 'Color', C2, 'LineWidth', 1.1*lw);
un_m = line(t, hp_drone(3,:));
set(un_m, 'LineStyle', '--', 'Color', C15, 'LineWidth', 1.1*lw);
un_ref = line(t, vref(3, :));
set(un_ref, 'LineStyle', '-.', 'Color', C9, 'LineWidth', 1.3*lw);


%% Title of the image
%xlabel('$\textrm{Time}[s]$','fontsize',10,'interpreter','latex','Color',C18);
ylabel('$[m/s]$','fontsize',9,'interpreter','latex', 'Color',C18);

%% Legend nomeclature
hLegend_3 = legend([un, un_m, un_ref],{'$\mu_{n}$','$\mu_{nm}$','$\mu_{nref}$'},'fontsize',11,'interpreter','latex','Color',[255 255 255]/255,'NumColumns',1,'TextColor','black');
set(gca,'ticklabelinterpreter','latex',...
    'fontsize',1.1*fontsizeTicks)
%% Figure properties
ax_3 = gca;
ax_3.Box = 'on';
ax_3.BoxStyle = 'full';
ax_3.TickLength = [0.01;0.01];
ax_3.TickDirMode = 'auto';
ax_3.XTickLabel = [];
ax_3.YMinorTick = 'on';
ax_3.XMinorTick = 'on';
ax_3.XMinorGrid = 'on';
ax_3.YMinorGrid = 'on';
ax_3.MinorGridAlpha = 0.15;
ax_3.LineWidth = 0.8;
ax_3.XLim = [0 t(end)];
% 
%% Figure 4 W
axes('Position',[0.04 0.16 .45 .17]);
%% FRONTAL VELOCITY
w = line(t, v(4, :));
set(w, 'LineStyle', '-', 'Color', C16, 'LineWidth', 1.1*lw);
w_m = line(t, hp_drone(4,:));
set(w_m, 'LineStyle', '--', 'Color', C17, 'LineWidth', 1.1*lw);
w_ref = line(t, vref(4, :));
set(w_ref, 'LineStyle', '-.', 'Color', C9, 'LineWidth', 1.3*lw);


%% Title of the image
%xlabel('$\textrm{Time}[s]$','fontsize',10,'interpreter','latex','Color',C18);
ylabel('$[rad/s]$','fontsize',9,'interpreter','latex', 'Color',C18);

%% Legend nomeclature
hLegend_4 = legend([w, w_m, w_ref],{'$\omega$','$\omega_m$','$\omega_{ref}$'},'fontsize',11,'interpreter','latex','Color',[255 255 255]/255,'NumColumns',1,'TextColor','black');
set(gca,'ticklabelinterpreter','latex',...
    'fontsize',1.1*fontsizeTicks)
%% Figure properties
ax_4 = gca;
ax_4.Box = 'on';
ax_4.BoxStyle = 'full';
ax_4.TickLength = [0.01;0.01];
ax_4.TickDirMode = 'auto';
ax_4.XTickLabel = [];
ax_4.YMinorTick = 'on';
ax_4.XMinorTick = 'on';
ax_4.XMinorGrid = 'on';
ax_4.YMinorGrid = 'on';
ax_4.MinorGridAlpha = 0.15;
ax_4.LineWidth = 0.8;
ax_4.XLim = [0 t(end)];
% 
%% Figure 5 q1p
axes('Position',[0.53 0.79 .45 .17]);
%% FRONTAL VELOCITY
q1p = line(t, qp(1, :));
set(q1p, 'LineStyle', '-', 'Color', C11, 'LineWidth', 1.1*lw);
q1p_m = line(t, qp_drone(1, :));
set(q1p_m, 'LineStyle', '--', 'Color', C12, 'LineWidth', 1.1*lw);
q1p_ref = line(t, qp_ref(1, :));
set(q1p_ref, 'LineStyle', '-.', 'Color', C9, 'LineWidth', 1.3*lw);


%% Title of the image
%xlabel('$\textrm{Time}[s]$','fontsize',10,'interpreter','latex','Color',C18);
ylabel('$[rad/s]$','fontsize',9,'interpreter','latex', 'Color',C18);

%% Legend nomeclature
hLegend_5 = legend([q1p, q1p_m, q1p_ref],{'$\dot{q}_1$', '$\dot{q}_{1m}$','$\dot{q}_{1ref}$'},'fontsize',11,'interpreter','latex','Color',[255 255 255]/255,'NumColumns',1,'TextColor','black');
set(gca,'ticklabelinterpreter','latex',...
    'fontsize',1.1*fontsizeTicks)
%% Figure properties
ax_5 = gca;
ax_5.Box = 'on';
ax_5.BoxStyle = 'full';
ax_5.TickLength = [0.01;0.01];
ax_5.TickDirMode = 'auto';
ax_5.XTickLabel = [];
ax_5.YMinorTick = 'on';
ax_5.XMinorTick = 'on';
ax_5.XMinorGrid = 'on';
ax_5.YMinorGrid = 'on';
ax_5.MinorGridAlpha = 0.15;
ax_5.LineWidth = 0.8;
ax_5.XLim = [0 t(end)];

%% Figure 6 q2p
axes('Position',[0.53 0.58 .45 .17]);
%% FRONTAL VELOCITY
q2p = line(t, qp(2, :));
set(q2p, 'LineStyle', '-', 'Color', C3, 'LineWidth', 1.1*lw);
q2p_m = line(t, qp_drone(2, :));
set(q2p_m, 'LineStyle', '--', 'Color', C14, 'LineWidth', 1.1*lw);
q2p_ref = line(t, qp_ref(2, :));
set(q2p_ref, 'LineStyle', '-.', 'Color', C9, 'LineWidth', 1.3*lw);


%% Title of the image
%xlabel('$\textrm{Time}[s]$','fontsize',10,'interpreter','latex','Color',C18);
ylabel('$[rad/s]$','fontsize',9,'interpreter','latex', 'Color',C18);

%% Legend nomeclature
hLegend_6 = legend([q2p, q2p_m, q2p_ref],{'$\dot{q}_2$', '$\dot{q}_{2m}$','$\dot{q}_{2ref}$'},'fontsize',11,'interpreter','latex','Color',[255 255 255]/255,'NumColumns',1,'TextColor','black');
set(gca,'ticklabelinterpreter','latex',...
    'fontsize',1.1*fontsizeTicks);
%% Figure properties
ax_6 = gca;
ax_6.Box = 'on';
ax_6.BoxStyle = 'full';
ax_6.TickLength = [0.01;0.01];
ax_6.TickDirMode = 'auto';
ax_6.XTickLabel = [];
ax_6.YMinorTick = 'on';
ax_6.XMinorTick = 'on';
ax_6.XMinorGrid = 'on';
ax_6.YMinorGrid = 'on';
ax_6.MinorGridAlpha = 0.15;
ax_6.LineWidth = 0.8;
ax_6.XLim = [0 t(end)];
% 
%% Figure 6 q2p
axes('Position',[0.53 0.37 .45 .17]);
%% FRONTAL VELOCITY
q3p = line(t, qp(3, :));
set(q3p, 'LineStyle', '-', 'Color', C2, 'LineWidth', 1.1*lw);
q3p_m = line(t, qp_drone(3, :));
set(q3p_m, 'LineStyle', '--', 'Color', C15, 'LineWidth', 1.1*lw);
q3p_ref = line(t, qp_ref(3, :));
set(q3p_ref, 'LineStyle', '-.', 'Color', C9, 'LineWidth', 1.3*lw);


%% Title of the image
%xlabel('$\textrm{Time}[s]$','fontsize',10,'interpreter','latex','Color',C18);
ylabel('$[rad/s]$','fontsize',9,'interpreter','latex', 'Color',C18);

%% Legend nomeclature
hLegend_7 = legend([q3p, q3p_m, q3p_ref],{'$\dot{q}_3$', '$\dot{q}_{3m}$', '$\dot{q}_{3ref}$'},'fontsize',11,'interpreter','latex','Color',[255 255 255]/255,'NumColumns',1,'TextColor','black');
set(gca,'ticklabelinterpreter','latex',...
    'fontsize',1.1*fontsizeTicks)
%% Figure properties
ax_7 = gca;
ax_7.Box = 'on';
ax_7.BoxStyle = 'full';
ax_7.TickLength = [0.01;0.01];
ax_7.TickDirMode = 'auto';
ax_7.XTickLabel = [];
ax_7.YMinorTick = 'on';
ax_7.XMinorTick = 'on';
ax_7.XMinorGrid = 'on';
ax_7.YMinorGrid = 'on';
ax_7.MinorGridAlpha = 0.15;
ax_7.LineWidth = 0.8;
ax_7.XLim = [0 t(end)];

