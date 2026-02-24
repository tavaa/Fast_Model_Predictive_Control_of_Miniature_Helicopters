% CONTROLPARAMS Parameters of the MPC Controller and Cost Function
% Defines the optimization horizon, sampling rate, and weighting matrices.
%
% Q_diag >= 0 (Positive semi-definite state cost).
% R_diag > 0 (Positive definite input cost for solver stability).
classdef ControlParams
    
    properties (Constant)
        % Timing Configuration
        Ts = 0.20;      % Sampling period [s] (50 Hz control frequency)
        p  = 18;        % Prediction horizon [steps] (0.36s look-ahead)
        
        % State Weighting Matrix Q (Diagonal entries)
        % Vector: [x, y, z, psi, dx, dy, dz, dpsi, xi, yi]
        % Higher weights for position (x, y) ensure tight tracking.

        % paper configuration
        Q_diag = [50; 50; 50; 10; 3; 3; 1; 2; 15; 15];

        % improved configuration
        %Q_diag = [500; 500; 50; 10; 3; 3; 1; 2; 15; 15];
        
        % Input Weighting Matrix R (Diagonal entries)
        % Vector: [ux, uy, uz, uPsi]

        % paper configuration
        R_diag = [2.0; 2.0; 2.0; 2.0];    

        % improved configuration
        %R_diag = [0.1; 0.1; 0.1; 0.1];
        
        % Terminal Cost Strategy
        % true: Solves Discrete Algebraic Riccati Equation (DARE) for Qf.
        % false: Sets Qf = Q.
        use_computed_terminal_cost = false; 

        % PID Benchmark Parameters
        % Gains are mapped to: [Pitch/X, Roll/Y, Thrust/Z, Yaw/Psi]

        % Standard Gains
        %PID_Kp = [2.0; 2.0; 5.0; 2.0];
        %PID_Ki = [0.5; 0.5; 2.0; 0.1]; 
        %PID_Kd = [1.5; 1.5; 2.5; 0.5];
        
        % High-performance Gains
        PID_Kp = [6.0; 6.0; 10.0; 3.0];
        PID_Ki = [3.5; 3.5; 8.0;  0.5]; 
        PID_Kd = [3.0; 3.0; 5.0;  1.0];
        
        % Anti-windup clamping thresholds for the integral term contribution
        PID_anti_windup_lim = [2.0; 2.0; 2.0; 1.0];
    end
end