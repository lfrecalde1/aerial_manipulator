function [u] = control_system(hd, hdp, h, q, L)
%% JACOBIAN MATRIX
[J] = jacobian_control(h, q, L);

%% CONTROL ERROR
h = h(1:3);
he = hd - h;

%% CONTROL LAW
u = pinv(J)*(hdp+tanh(he));

end

