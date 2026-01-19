% TRAJECTORYBASE Abstract Interface for Differentially Flat Trajectory Generation
% This class serves as the base interface for defining feasible paths for the 
% miniature helicopter. It enforces a structure compatible with the 
% Inverse Dynamics Engine (FlatnessMap).

classdef (Abstract) TrajectoryBase < handle
    properties
        % params: Configuration structure containing geometric and temporal constants.
        % Must be initialized with a valid struct from TrajectoryParams.
        params 
    end
    
    methods
        %% Constructor: Initializes the trajectory with specific geometric parameters.
        %
        % Args:
        %   params - Parameter structure (e.g., Hover, Circle, or Spiral).
        function obj = TrajectoryBase(params)
            obj.params = params;
        end
    end
    
    methods (Abstract)
        %% GET_FLAT_OUTPUTS Computes the flat output state and derivatives at time t.
        %
        % Args:
        %   t - Scalar simulation/real-world time [s].
        %
        % Returns:
        %   z   - Flat output vector [xI; yI; zI; Psi] in R^4.
        %   dz  - First derivative (Velocity vector) [vxI; vyI; vzI; dPsi] in R^4.
        %   ddz - Second derivative (Acceleration vector) [axI; ayI; azI; ddPsi] in R^4.
        %
        % Preconditions:
        %   t >= 0.
        % Postconditions:
        %   size(z) == size(dz) == size(ddz) == [4, 1].
        [z, dz, ddz] = get_flat_outputs(obj, t);
    end
end