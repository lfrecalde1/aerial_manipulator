classdef mobile_robot_dynamic < matlab.mixin.SetGet
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % Distance to the center of axis
        a
        
        % Dynamic parameters of the system
        chi
        
        % General vector of the system
        q
        
        % System sample time
        ts
    end
    
    methods
        function obj = mobile_robot_dynamic(L, chi, x, ts)
            
            % Propierties definition of the system 
            obj.a = L(1);
            
            obj.q = x;
             
            obj.chi = chi;
            
            obj.ts = ts;
            
        end
        
        function J = J_matrix(obj, x)
            %Gets the Jacobian Matrix
            %   Split values of the states
            q_x = x(1);
            q_y = x(2);
            theta = x(3);
            
            % get Jacobian Matrix of the system
            
            J_11 = cos(theta);
            J_12 = -obj.a*sin(theta);
            
            J_21 = sin(theta);
            J_22 = obj.a*cos(theta);
            
            J_31 = 0;
            J_32 = 1;
            
            J = [J_11, J_12;...
                 J_21, J_22;...
                 J_31, J_32];
        end
        function J = get_J_matrix_control(obj)
            %Gets the Jacobian Matrix
            %   Split values of the states
            
            q_x = obj.q(1);
            q_y = obj.q(2);
            theta = obj.q(3);
            
            % get Jacobian Matrix of the system
            
            J_11 = cos(theta);
            J_12 = -obj.a*sin(theta);
            
            J_21 = sin(theta);
            J_22 = obj.a*cos(theta);
            
            J_31 = 0;
            J_32 = 1;
            
            J = [J_11, J_12;...
                 J_21, J_22];
        end
        
        function M = get_M_matrix(obj)
            
            % get states of the system
            q_x = obj.q(1);
            q_y = obj.q(2);
            theta = obj.q(3);
            q_u = obj.q(4);
            q_w = obj.q(5);
            
            % Dynamic parameters
            chi_1 = obj.chi(1);
            chi_2 = obj.chi(2);
            chi_3 = obj.chi(3);
            chi_4 = obj.chi(4);
            chi_5 = obj.chi(5);
            chi_6 = obj.chi(6);
            
            M11 = chi_1;
            M12 = 0;
            M21 = 0;
            M22 = chi_2;
            
            M = [M11, M12;...
                 M21, M22];
            
        end
        
        function C = get_C_matrix(obj)
            
            % get states of the system
            q_x = obj.q(1);
            q_y = obj.q(2);
            theta = obj.q(3);
            q_u = obj.q(4);
            q_w = obj.q(5);
            
            % Dynamic parameters
            chi_1 = obj.chi(1);
            chi_2 = obj.chi(2);
            chi_3 = obj.chi(3);
            chi_4 = obj.chi(4);
            chi_5 = obj.chi(5);
            chi_6 = obj.chi(6);
            
            C1 = -chi_3*q_w^2 + chi_4*q_u;
            C2 = chi_5*q_u*q_w + chi_6*q_w;
            
            C = [C1;...
                 C2];
            
        end
        
        function xp = f_model(obj, x, u)
            %Gets funtion dot of the system
            
            % Get Jacobian matrix
            J = obj.J_matrix(x);
            
            % States of the system
            q_x = x(1);
            q_y = x(2);
            theta = x(3);
            q_u = x(4);
            q_w = x(5);
            
            % Dynamic parameters
            chi_1 = obj.chi(1);
            chi_2 = obj.chi(2);
            chi_3 = obj.chi(3);
            chi_4 = obj.chi(4);
            chi_5 = obj.chi(5);
            chi_6 = obj.chi(6);
            
            % Get system Matrices
            f = [q_u*cos(theta) - obj.a*sin(theta)*q_w;...
                 q_u*sin(theta) + obj.a*cos(theta)*q_w;...
                 q_w;...
                 (chi_3/chi_1)*q_w^2 - (chi_4/chi_1)*q_u;...
                -(chi_5/chi_2)*q_u*q_w - (chi_6/chi_2)*q_w];
            
            g = [0, 0;...
                 0, 0;...
                 0, 0;...
                 1/chi_1, 0;...
                 0, 1/chi_2];
             
            xp = f + g*u;
        end
        
        function [x] = system_f(obj, u)
            % Sample Time
            T_s = obj.ts;
            
            % General States System
            x = obj.q;
            
            k1 = obj.f_model(x, u);
            k2 = obj.f_model(x + T_s/2*k1, u);
            k3 = obj.f_model(x + T_s/2*k2, u);
            k4 = obj.f_model(x + T_s*k3, u);
            x = x +T_s/6*(k1 +2*k2 +2*k3 +k4);
            
            % Update values system
            obj.q = x;
        end
        
        function x = get_states(obj)
            % Get system states
            x = obj.q;
        end
    end
end
