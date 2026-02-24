classdef ShapeLemniscate < TrajectoryBase
    % SHAPELEMNISCATE Implementation of a 3D Lemniscate (Figure-8) Trajectory
    
    methods
        function obj = ShapeLemniscate(lemniscate_params)
            obj@TrajectoryBase(lemniscate_params);
        end
        
        function [z, dz, ddz] = get_flat_outputs(obj, t)

            p = obj.params;
            R = p.R;         % lemniscate radius
            w = p.omega;     % angular velocity
            zh = p.z_height; % height

            % center of figure
            cx = p.center(1);
            cy = p.center(2);
            
            % Pre-computed terms
            A = R * sqrt(2);
            theta = w * t;
            sin_t  = sin(theta);
            cos_t  = cos(theta);
            sin2_t = sin_t^2;
            den = sin2_t + 1; % (sin(wt)^2 + 1) 
            
            %% POSITION (z1, z2, z3)
            z1 = (A * cos_t / den) + cx;          % x
            z2 = (A * cos_t * sin_t / den) + cy;  % y
            z3 = zh;                              % h
            
            %% VELOCITY (dz1, dz2, dz3)
            dz1 = A * w * (sin_t^3 - 3 * sin_t) / (den^2); 
            dz2 = A * w * (1 - 3 * sin2_t) / (den^2); 
            dz3 = 0; 
            
            %% ACCELERATION (ddz1, ddz2, ddz3)
            ddz1 = A * w^2 * cos_t * (-3 + 12 * sin2_t - sin_t^4) / (den^3); 
            ddz2 = 2 * A * w^2 * sin_t * cos_t * (3 * sin2_t - 5) / (den^3);
            ddz3 = 0; 
            
            %% YAW (z4) - Tangent Mode
            z4 = atan2(dz2, dz1); 
            
            % dz4: Yaw Rate
            % (dz1*ddz2 - dz2*ddz1) / (dz1^2 + dz2^2)
            vel_sq = dz1^2 + dz2^2; 
            num_dz4 = (dz1 * ddz2) - (dz2 * ddz1);
            
            % Handle exception null denom.
            if vel_sq > 1e-6
                dz4 = num_dz4 / vel_sq;
            else
                dz4 = 0;
            end
            
            %% YAW ACCELERATION (ddz4)

            % calculate 3rd derivatives (dddz1, dddz2):
            dddz1 = A * w^3 * (-sin_t^7 + 43 * sin_t^5 - 103 * sin_t^3 + 45 * sin_t) / (den^4);
            dddz2 = A * w^3 * (12 * sin_t^6 - 82 * sin_t^4 + 88 * sin2_t - 10) / (den^4);

            if vel_sq > 1e-6
                dot_N = (dz1 * dddz2 - dz2 * dddz1);
                dot_V = (2 * dz1 * ddz1 + 2 * dz2 * ddz2);
                
                ddz4 = (dot_N * vel_sq - num_dz4 * dot_V) / (vel_sq^2);
            else
                ddz4 = 0;
            end

            % Vector Assembly 
            z   = [z1; z2; z3; z4];
            dz  = [dz1; dz2; dz3; dz4];
            ddz = [ddz1; ddz2; ddz3; ddz4];
        end
    end
end