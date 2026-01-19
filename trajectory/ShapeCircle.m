% SHAPECIRCLE Implementation of a Planar Circular Trajectory
% This class generates a circular flight path in the XY plane at a constant 
% altitude. The heading (yaw) can be configured to either follow the 
% velocity vector or remain fixed at a specific orientation.

classdef ShapeCircle < TrajectoryBase
    
    methods
        %% Constructor: Initializes circular path parameters from the global config.
        %
        % Args:
        %   circle_params - Struct containing {R, omega, z_height, yaw_mode}.
        function obj = ShapeCircle(circle_params)
            obj@TrajectoryBase(circle_params);
        end
        
        %% GET_FLAT_OUTPUTS Computes the circular kinematic state and analytic derivatives.
        %
        % Args:
        %   t - Elapsed time [s].
        %
        % Returns:
        %   z   - Flat output vector [xI; yI; zI; Psi].
        %   dz  - First derivative vector (Velocities).
        %   ddz - Second derivative vector (Accelerations).
        function [z, dz, ddz] = get_flat_outputs(obj, t)
            % Extract parameters for readable calculation
            R = obj.params.R;
            w = obj.params.omega;
            h = obj.params.z_height;
            
            % XY Plane: Circular Motion 
            % Position based on radius R and angular velocity w.
            z1 = R * cos(w * t);
            z2 = R * sin(w * t);
            
            % Velocity 
            dz1 = -R * w * sin(w * t);
            dz2 =  R * w * cos(w * t);
            
            % Acceleration 
            ddz1 = -R * w^2 * cos(w * t);
            ddz2 = -R * w^2 * sin(w * t);
            
            % Z Axis: Constant Altitude 
            z3   = h;
            dz3  = 0;
            ddz3 = 0;
            
            % Yaw Axis
            if strcmp(obj.params.yaw_mode, 'tangent')
                % Coordinated Turn Logic:
                % The heading is aligned with the velocity vector: atan2(dy, dx).
                % This results in a persistent phase shift of pi/2 relative to position.
                z4   = w * t + pi/2;
                dz4  = w;
                ddz4 = 0;
            else
                % Fixed Heading: Static orientation (Regulation on Yaw)
                z4   = 0;
                dz4  = 0;
                ddz4 = 0;
            end
            
            % Vector Assembly 
            z   = [z1; z2; z3; z4];
            dz  = [dz1; dz2; dz3; dz4];
            ddz = [ddz1; ddz2; ddz3; ddz4];
        end
    end
end