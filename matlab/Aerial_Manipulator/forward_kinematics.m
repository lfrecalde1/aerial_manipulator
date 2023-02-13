function [h] =  forward_kinematics(h_drone, q_arm, L)

%% GET INTERNAL STATES
x_0 = h_drone(1);
y_0 = h_drone(2);
z_0 = h_drone(3);
psi = h_drone(4);

q_1 = q_arm(1);
q_2 = q_arm(2);
q_3 = q_arm(3);


l_1 = L(1);
l_2 = L(2);
l_3 = L(3);

%% FORWARD KINEMATICS
hx1 =  x_0 + cos(psi + q_1)*(l_3*cos(q_2 + q_3) + l_2*cos(q_2));
hy1 =  y_0 + sin(psi + q_1)*(l_3*cos(q_2 + q_3) + l_2*cos(q_2));
hz1 =  z_0 - l_1 - l_2*sin(q_2) - l_3*sin(q_2 + q_3);

h = [hx1;hy1;hz1;psi];

end

