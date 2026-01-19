% SHAPEHOVER Implementation of a Stationary Setpoint Trajectory
% This class generates a constant flat output reference for regulation problems,
% representing a static hovering condition at a fixed position and heading.

classdef ShapeHover < TrajectoryBase
    
    methods
        %% Constructor: Initializes the static regulation setpoint.
        %
        % Args:
        %   hover_params - Struct containing {pos, psi}, where pos is [x; y; z].
        function obj = ShapeHover(hover_params)
            obj@TrajectoryBase(hover_params);
        end
        
        %% GET_FLAT_OUTPUTS Computes the static state and null derivatives.
        %
        % Args:
        %   t - Elapsed time [s] (unused for static setpoints).
        %
        % Returns:
        %   z   - Constant flat output vector [xI; yI; zI; Psi].
        %   dz  - Zero vector (Null velocities).
        %   ddz - Zero vector (Null accelerations).
        %
        % Postconditions:
        %   norm(dz) == 0, norm(ddz) == 0 for all t >= 0.
        function [z, dz, ddz] = get_flat_outputs(obj, ~)
            % Extract persistent setpoint from parameters
            z = [obj.params.pos; obj.params.psi];
            
            % Regulation assumes zero equilibrium velocities and accelerations
            dz  = zeros(4, 1);
            ddz = zeros(4, 1);
        end
    end
end