% MODELPARAMS Physical Parameters of the Helicopter Model
% This class encapsulates the identified aerodynamic coefficients and safety limits.
%
% b_i coefficients must be positive for system controllability.
% u_min <= u_max (Saturation constraints).
classdef ModelParams

    properties (Constant)
        % Physical Constants
        g = 9.81; % Gravitational acceleration [m/s^2]
        
        % Aerodynamic Input Coefficients 
        bx = 2.0;   % Pitch effectiveness
        by = 2.1;   % Roll effectiveness
        bz = 18.0;  % Thrust effectiveness
        bPsi = 111.0; % Yaw effectiveness

        % Damping and Drag Coefficients
        kx = -0.5;
        ky = -0.4;
        kPsi = -5.0;
        
        % Integral Gain: Minimizes steady-state error in the LTV-MPC formulation.
        ki = 2.0; 
        
        % State Validity Constraints: Bounds for Linear Time-Varying model reliability.
        % ||v_body|| <= v_max [m/s].
        v_max = [3.0; 3.0; 2.0]; 

        % Yaw Rate Constraint: Maximum angular velocity.
        % |d_psi/dt| <= psi_rate_max [rad/s].
        psi_rate_max = 25.0; 
        
        % Input Saturation Limits [Pitch, Roll, Thrust, Yaw]
        % u in [u_min, u_max].
        u_min = [-1.0; -1.0; -0.4; -1.0]; 
        u_max = [ 1.0;  1.0;  1.0;  1.0]; 
    end
end