% SHAPESPIRAL Implementation of a 3D Helical Trajectory
% This class generates an ascending spiral trajectory. Heading (Psi) logic 
% is switched based on 'yaw_mode' to support either tangent orientation 
% or constant velocity pirouettes.
%
% The trajectory is C^2 continuous.
% XY projection returns to the starting point for multiples of 2*pi/omega.
classdef ShapeSpiral < TrajectoryBase
    
    methods
        %% Constructor: Initializes the helical path parameters.
        %
        % Args:
        %   spiral_params - Struct containing {R, omega, vz, z_start, Omega_yaw, yaw_mode}.
        function obj = ShapeSpiral(spiral_params)
            obj@TrajectoryBase(spiral_params);
        end
        
        %% GET_FLAT_OUTPUTS Computes the kinematic state and analytic derivatives at time t.
        %
        % Args:
        %   t - Elapsed time [s].
        %
        % Returns:
        %   z   - Flat output vector [xI; yI; zI; Psi].
        %   dz  - First derivative vector (Velocities).
        %   ddz - Second derivative vector (Accelerations).
        function [z, dz, ddz] = get_flat_outputs(obj, t)
            p = obj.params;
            
            % XY Plane: Circular Motion Component 
            % x = R cos(wt), y = R sin(wt)
            z1 = p.R * cos(p.omega * t);
            z2 = p.R * sin(p.omega * t);
            
            dz1 = -p.R * p.omega * sin(p.omega * t);
            dz2 =  p.R * p.omega * cos(p.omega * t);
            
            ddz1 = -p.R * p.omega^2 * cos(p.omega * t);
            ddz2 = -p.R * p.omega^2 * sin(p.omega * t);
            
            % Z Axis: Constant Vertical Velocity Component 
            z3   = p.z_start + p.vz * t;
            dz3  = p.vz;
            ddz3 = 0;
            
            % Yaw Axis: Heading Strategy Selection 
            if strcmp(p.yaw_mode, 'tangent')
                % Coordinated Turn: Yaw is aligned with the XY velocity vector.
                % atan2(dy, dx) leads to a constant pi/2 shift from the position angle.
                z4   = p.omega * t + pi/2;
                dz4  = p.omega;
                ddz4 = 0;
            else
                % Pirouette Mode: Yaw rotates at an independent constant rate Omega_yaw.
                z4   = p.Omega_yaw * t;
                dz4  = p.Omega_yaw;
                ddz4 = 0;
            end
            
            % Vector Assembly 
            z   = [z1; z2; z3; z4];
            dz  = [dz1; dz2; dz3; dz4];
            ddz = [ddz1; ddz2; ddz3; ddz4];
        end
    end
end