function [v_final] = frame_transformation(h, hp, t)
%% INTERNAL STATES DRONE
x = h(1, :);
y = h(2, :);
z = h(3, :);
yaw = h(4, :);

%% NEW VELOCITY BODY FRAME
v = zeros(4, length(t));
v_final = zeros(4, length(t));
for k = 1:length(t)
    %% GET JACOBIAN MATRIX
    J11 = cos(yaw(k));
    J12 = -sin(yaw(k));
    J13 = 0;
    J14 = 0;
    
    
    J21 = sin(yaw(k));
    J22 = cos(yaw(k));
    J23 = 0;
    J24 = 0;
    
    J31 = 0;
    J32 = 0;
    J33 = 1;
    J34 = 0;
    
    J41 = 0;
    J42 = 0;
    J43 = 0;
    J44 = 1;
    
    J = [J11, J12, J13, J14;...
         J21, J22, J23, J24;...
         J31, J32, J33, J34;...
         J41, J42, J43, J44];
     
    %% GET NEW VELOCITIES
    v(:, k) = inv(J)*hp(:, k);
    
end
    %% FIX VALUES 
    v_final(1, :) = v(2, :);
    v_final(2, :) = -v(1, :);
    v_final(3, :) = v(3, :);
    v_final(4, :) = v(4, :);
end

