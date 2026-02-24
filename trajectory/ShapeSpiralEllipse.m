classdef ShapeSpiralEllipse < TrajectoryBase
    % SHAPESPIRALELLIPSE Implementation of a 3D Elliptical Helix Trajectory
    %
    
    methods
        function obj = ShapeSpiralEllipse(ellipse_params)
            obj@TrajectoryBase(ellipse_params);
        end
        
        function [z, dz, ddz] = get_flat_outputs(obj, t)

            p = obj.params;
            w = p.omega;
            sin_wt = sin(w * t);
            cos_wt = cos(w * t);
            
            %% POSITION (x, y, z)
            x = p.Rx * cos_wt;
            y = p.Ry * sin_wt;
            h = p.z_start + p.vz * t;
            
            % Velocity (d/dt)
            dx = -p.Rx * w * sin_wt;
            dy =  p.Ry * w * cos_wt;
            dh =  p.vz;
            
            % Acceleration (d^2/dt^2)
            ddx = -p.Rx * w^2 * cos_wt;
            ddy = -p.Ry * w^2 * sin_wt;
            ddh = 0;
            
            %% YAW (psi) - Tangent Mode
            psi = atan2(dy, dx);
            
            % dz4: Yaw Rate
            % Formula: (dx*ddy - dy*ddx) / (dx^2 + dy^2)
            % Numerator simplifies to: Rx * Ry * w^3
            vel_sq = dx^2 + dy^2; % Speed squared
            num    = p.Rx * p.Ry * w^3; 
            
            dpsi = num / vel_sq;
            
            % ddz4: Yaw Acceleration
            % Derivative of (Const / vel_sq) -> -Const * (d(vel_sq)/dt) / (vel_sq)^2
            % d(vel_sq)/dt = 2*dx*ddx + 2*dy*ddy
            
            d_vel_sq = 2*dx*ddx + 2*dy*ddy;
            ddpsi    = -(num * d_vel_sq) / (vel_sq^2);

            %% ASSEMBLY
            z   = [x; y; h; psi];
            dz  = [dx; dy; dh; dpsi];
            ddz = [ddx; ddy; ddh; ddpsi];
        end
    end
end