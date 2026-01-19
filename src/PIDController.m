% PIDCONTROLLER Classical Benchmark SISO Control Architecture
%
% Purpose:
%   Implements four decoupled Proportional-Integral-Derivative control loops 
%   for autonomous helicopter tracking.
%
% Logic:
%   1. Error Wrapping: shortest-path logic for angular heading (Yaw).
%   2. Coordination: Error rotation into the Body Frame (Inertial-to-Body).
%   3. Regulation: Classic PID law with discrete derivative approximation.
%   4. Robustness: Clamping Anti-Windup for integral saturation management.
%   5. Linearization: Gravity feedforward to simplify vertical control.

classdef PIDController < handle
    
    properties
        mp              % ModelParams: Physical system constants
        cp              % ControlParams: Controller configuration and gains
        
        Kp, Ki, Kd      % Gain vectors [4x1]
        prev_error      % Error memory for finite difference derivative
        integral_error  % Accumulated state error for integral action
        anti_windup_lim % Integral term saturation thresholds
    end
    
    methods
        %% CONSTRUCTOR: Initializes controller weights and memory.
        %
        % Args:
        %   gains_struct - (Optional) Structure containing Kp, Ki, Kd vectors.
        %                  Defaults to ControlParams if omitted.
        function obj = PIDController(gains_struct)
            obj.mp = ModelParams;
            obj.cp = ControlParams;
            
            % Dynamic gain assignment
            if nargin > 0 && isstruct(gains_struct)
                obj.Kp = gains_struct.Kp;
                obj.Ki = gains_struct.Ki;
                obj.Kd = gains_struct.Kd;
            else
                obj.Kp = obj.cp.PID_Kp;
                obj.Ki = obj.cp.PID_Ki;
                obj.Kd = obj.cp.PID_Kd;
            end
            
            obj.anti_windup_lim = obj.cp.PID_anti_windup_lim;
            obj.reset();
        end
        
        %% RESET: Resets internal integrators and error state memory.
        %
        % Postconditions:
        %   prev_error = [0;0;0;0], integral_error = [0;0;0;0].
        function reset(obj)
            obj.prev_error = zeros(4, 1);
            obj.integral_error = zeros(4, 1);
        end
        
        %% COMPUTE: Executes the decoupled control law.
        %
        % Args:
        %   x_meas - Current state feedback [10x1].
        %   x_ref  - Reference state command [10x1].
        %
        % Returns:
        %   u - Saturated control inputs [4x1].
        %   debug_info - Diagnostic (errors and integral states).
        function [u, debug_info] = compute(obj, x_meas, x_ref)
            
            % Inertial Error Calculation & Angular Wrapping 
            err_inertial = x_ref(1:4) - x_meas(1:4);
            % Ensure yaw error adheres to the shortest angular path via atan2
            err_inertial(4) = atan2(sin(err_inertial(4)), cos(err_inertial(4)));
            
            % Body Frame Projection (Inertial-to-Body)
            % Transforms inertial position error into local pitch/roll axes.
            Psi = x_meas(4);
            c = cos(Psi); s = sin(Psi);
            
            err_body_x =  c * err_inertial(1) + s * err_inertial(2);
            err_body_y = -s * err_inertial(1) + c * err_inertial(2);
            
            % Aggregated error vector in Body-Fixed Frame
            err_vec = [err_body_x; err_body_y; err_inertial(3); err_inertial(4)];
            
            % Discrete PID Law Implementation 
            % Integral accumulation with Euler forward integration
            obj.integral_error = obj.integral_error + err_vec * obj.cp.Ts;
            int_term = obj.Ki .* obj.integral_error;
            
            % Anti-Windup Clamping: Prevents integral saturation during motor limits
            int_term = max(min(int_term, obj.anti_windup_lim), -obj.anti_windup_lim);
            
            % Numerical derivative calculation (First-order backward difference)
            deriv_vec = (err_vec - obj.prev_error) / obj.cp.Ts;
            
            % Summation of Proportional, Integral, and Derivative actions
            u_pid = (obj.Kp .* err_vec) + int_term + (obj.Kd .* deriv_vec);
            
            % Linearization & Actuator Saturation
            
            % Gravity Feedforward: Bias required for hover equilibrium (u = g/bz)
            u_hover = obj.mp.g / obj.mp.bz;
            u_ff = [0; 0; u_hover; 0];
            
            u_raw = u_pid + u_ff;
            
            % Enforce normalized input saturation limits defined in ModelParams
            u = max(min(u_raw, obj.mp.u_max), obj.mp.u_min);
            
            % Memory synchronization for the next sampling period
            obj.prev_error = err_vec;
            
            % Export
            debug_info.err = err_vec;
            debug_info.int = obj.integral_error;
        end
    end
end