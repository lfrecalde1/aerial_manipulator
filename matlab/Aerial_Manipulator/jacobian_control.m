function [J] = jacobian_control(h, q, L)
%% GET INTERNAL VALUES
x_0 = h(1);
y_0 = h(2);
z_0 = h(3);
psi = h(4);

q_1 = q(1);
q_2 = q(2);
q_3 = q(3);

l_1 = L(1);
l_2 = L(2);
l_3 = L(3);

%% GET JACOBIAN MATRIX
J_11 = cos(psi);
J_12 = -sin(psi);
J_13 = 0;
J_14 = -sin(psi + q_1)*(l_3*cos(q_2 + q_3) + l_2*cos(q_2));
J_15 = -sin(psi + q_1)*(l_3*cos(q_2 + q_3) + l_2*cos(q_2));
J_16 = -cos(psi + q_1)*(l_3*sin(q_2 + q_3) + l_2*sin(q_2));
J_17 = -l_3*cos(psi + q_1)*sin(q_2 + q_3);

J_21 = sin(psi);
J_22 = cos(psi);
J_23 = 0;
J_24 = cos(psi + q_1)*(l_3*cos(q_2 + q_3) + l_2*cos(q_2));
J_25 = cos(psi + q_1)*(l_3*cos(q_2 + q_3) + l_2*cos(q_2));
J_26 = -sin(psi + q_1)*(l_3*sin(q_2 + q_3) + l_2*sin(q_2));
J_27 = -l_3*sin(psi + q_1)*sin(q_2 + q_3);

J_31 = 0;
J_32 = 0;
J_33 = 1;
J_34 = 0;
J_35 = 0;
J_36 = - l_3*cos(q_2 + q_3) - l_2*cos(q_2);
J_37 = -l_3*cos(q_2 + q_3);

%% CREATE MATRIX
J = [J_11, J_12, J_13, J_14, J_15, J_16, J_17;...
     J_21, J_22, J_23, J_24, J_25, J_26, J_27;...
     J_31, J_32, J_33, J_34, J_35, J_36, J_37];
end

