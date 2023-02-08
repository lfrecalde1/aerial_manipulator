function [h] = f_dynamic(x, v, vref, Ts)
%UNTITLED8 Summary of this function goes here
%   Detailed explanation goes here
h = v;
uref = vref;

k1 = system_dynamic(x, h, uref);
k2 = system_dynamic(x, h + (Ts/2)*k1, uref);
k3 = system_dynamic(x, h + (Ts/2)*k2, uref);
k4 = system_dynamic(x, h + Ts*k3, uref);
h = h + (Ts/6)*(k1 + 2*k2 + 2*k3 + k4);
end

