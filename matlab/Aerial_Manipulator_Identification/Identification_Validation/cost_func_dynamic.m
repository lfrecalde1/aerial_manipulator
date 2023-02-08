function [cost] = cost_func_dynamic(x, vref, vp, v, qpref, qpp, qp, q, N)
%UNTITLED6 Summary of this function goes here
vref_system = open_loop_dynamic(x, vp, v, qpp, qp, q, N);
he = error_dynamic(vref, qpref, vref_system, N);
cost = norm(he) + 0.0*norm(x,1);
end

