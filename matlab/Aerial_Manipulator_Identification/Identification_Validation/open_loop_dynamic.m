function [vref] = open_loop_dynamic(x, vp, v, qpp, qp, q, N)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
for k=1:N
   vref(:,k) = dynamic_identification(x, vp(:,k), v(:,k), qpp(:,k), qp(:,k), q(:,k)); 
end
end

