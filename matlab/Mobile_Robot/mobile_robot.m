classdef mobile_robot < matlab.mixin.SetGet
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
        function obj = mobile_robot(L, chi, x, ts)
            
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
        
        function J = J_matrix_control(obj,x)
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
                J_21, J_22];
        end
        
        function xp = f_model(obj, x, u)
            %Gets funtion dot of the system
            
            % Get Jacobian matrix
            J = obj.J_matrix(x);
            
            % Get system
            xp = J*u;
        end
        
        function [x] = system_f(obj, u)
            % Sample Time
            T_s = obj.ts;
            
            % General States System
            x = obj.q;
           
            k1 = obj.f_model(x, u)
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

