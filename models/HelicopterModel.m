classdef HelicopterModel
    % HELICOPTERMODEL Core physics and linearization engine.
    %
    % Implements the nonlinear ODE described in Eq. (1) of the paper:
    % "Fast Model Predictive Control of Miniature Helicopters" (Kunz et al.)
    %
    % State Vector (10x1):
    %   x = [xI, yI, zI, Psi, dxB, dyB, dzB, dPsi, x_int, y_int]'
    %   1-3:  Position (Inertial Frame)
    %   4:    Yaw Angle (Inertial)
    %   5-7:  Velocity (Body Frame)
    %   8:    Yaw Rate (Body/Inertial)
    %   9-10: Integral States (Position Error)
    %
    % Input Vector (4x1):
    %   u = [ux, uy, uz, uPsi]' (Pitch, Roll, Thrust, Yaw cmd)
    %
    %

    properties
        mp                % Instance of ModelParams (config)
        simplify_coupling % Boolean flag for Model Identification simplification
    end
    
    methods
        function obj = HelicopterModel(simplify_coupling_flag)
            % CONSTRUCTOR
            % simplify_coupling_flag: 
            %   true  -> Remove yaw-coupling in Jacobians 
            %   false -> Use full Jacobian (Exact math)
            
            obj.mp = ModelParams; 
            
            if nargin < 1
                obj.simplify_coupling = false; % Default to full physics
            else
                obj.simplify_coupling = simplify_coupling_flag;
            end
        end
        
        function dxdt = dynamics(obj, t, x, u, xref)
            % DYNAMICS Non-linear ODE implementation (Eq. 1)
            %
            % Inputs:
            %   t: Time (scalar)
            %   x: Current State (10x1)
            %   u: Current Input (4x1)
            %   xref: Reference state (10x1) [Optional, for integrals]
            
            % Unpack State
            Psi  = x(4);
            dxB  = x(5);
            dyB  = x(6);
            dzB  = x(7); 
            dPsi = x(8);
 
            p = obj.mp;
            
            % Kinematics (Body Velocity -> Inertial Velocity)
            % Rotation Matrix R_z(Psi)
            dxI = cos(Psi)*dxB - sin(Psi)*dyB;
            dyI = sin(Psi)*dxB + cos(Psi)*dyB;
            dzI = x(7); 
            
            % Dynamics (Accelerations in Body Frame)
            % Includes Drag (k*v) and Coriolis (dPsi*v)
            
            % Longitudinal (x)
            ddxB = p.bx * u(1) + p.kx * dxB + dPsi * dyB;
            
            % Lateral (y)
            ddyB = p.by * u(2) + p.ky * dyB - dPsi * dxB;
            
            % Vertical (z) 
            ddzB = p.bz * u(3) - p.g;
            
            % Rotational (Yaw)
            ddPsi = p.bPsi * u(4) + p.kPsi * dPsi;
            
            % Integral States Dynamics
            % d(x_int) = ki * (x_I - x_ref)
            if nargin < 5 || isempty(xref)
                % If no reference provided, assume regulation to 0 or hold constant
                err_x = x(1); 
                err_y = x(2);
            else
                err_x = x(1) - xref(1);
                err_y = x(2) - xref(2);
            end
            
            dxi = p.ki * err_x;
            dyi = p.ki * err_y;
            
            % Pack Derivative
            dxdt = [dxI; dyI; dzI; dPsi; ddxB; ddyB; ddzB; ddPsi; dxi; dyi];
        end
        
        function [A, B] = get_jacobians(obj, x_lin, u_lin)
            % GET_JACOBIANS Analytical Linearization (LTV Model)
            % Calculates A = df/dx and B = df/du at point (x_lin, u_lin).
            %
            % Implements the Simplification
            % if obj.simplify_coupling is true.
            
            % Unpack Linearization Point
            Psi  = x_lin(4);
            dxB  = x_lin(5);
            dyB  = x_lin(6);
            dPsi = x_lin(8);
            
            p = obj.mp;
            
            % Initialize Matrices
            nx = 10; nu = 4;
            A = zeros(nx, nx);
            B = zeros(nx, nu);
            
            %% MATRIX A (Jacobian df/dx) 
            
            % Kinematics Rows (1-3)
            % dxI = cos(Psi)dxB - sin(Psi)dyB
            A(1, 4) = -sin(Psi)*dxB - cos(Psi)*dyB; % d/dPsi
            A(1, 5) =  cos(Psi);                    % d/ddxB
            A(1, 6) = -sin(Psi);                    % d/ddyB
            
            % dyI = sin(Psi)dxB + cos(Psi)dyB
            A(2, 4) =  cos(Psi)*dxB - sin(Psi)*dyB; % d/dPsi
            A(2, 5) =  sin(Psi);                    % d/ddxB
            A(2, 6) =  cos(Psi);                    % d/ddyB
            
            % dzI = dzB
            A(3, 7) = 1;
            
            % Yaw Kinematics Row (4)
            A(4, 8) = 1; % dPsi/dPsi_dot
            
            % Dynamics Rows (5-7)
            % ddxB = bx*ux + kx*dxB + dPsi*dyB
            A(5, 5) = p.kx;       % d/dxB (Drag)
            
            if obj.simplify_coupling
                A(5, 6) = 0;      % Ignored Coriolis (dPsi)
                A(5, 8) = 0;      % Ignored Coupling (dyB)
            else
                A(5, 6) = dPsi;   % d/dyB (Coriolis)
                A(5, 8) = dyB;    % d/dPsi_dot (Coupling)
            end
            
            % Row 6: ddyB = by*uy + ky*dyB - dPsi*dxB
            A(6, 6) = p.ky;       % d/dyB (Drag)
            
            if obj.simplify_coupling
                A(6, 5) = 0;      % Ignored Coriolis (-dPsi)
                A(6, 8) = 0;      % Ignored Coupling (-dxB)
            else
                A(6, 5) = -dPsi;  % d/dxB
                A(6, 8) = -dxB;   % d/dPsi_dot
            end
            
            % Row 7: ddzB (Vertical)
            % No state dependency in simplified model (gravity is constant)
            
            % Rotational Dynamics Row (8)
            A(8, 8) = p.kPsi;
            
            % Integral States Rows (9-10)
            % d(xi)/dxI = ki
            A(9, 1)  = p.ki;
            A(10, 2) = p.ki;
            
            %% MATRIX B (Jacobian df/du) ---
            % Direct mapping from inputs to accelerations
            
            B(5, 1) = p.bx;   % Pitch input -> X Accel
            B(6, 2) = p.by;   % Roll input  -> Y Accel
            B(7, 3) = p.bz;   % Thrust      -> Z Accel
            B(8, 4) = p.bPsi; % Yaw input   -> Yaw Accel
            
        end
    end
end