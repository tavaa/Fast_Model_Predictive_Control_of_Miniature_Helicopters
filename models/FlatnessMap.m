% FLATNESSMAP Inverse Dynamics and State Reconstruction Engine
% This class implements the differential flatness mapping.
% It provides an algebraic mapping from the flat output space (trajectories in R^4) 
% to the full state-input space of the nonlinear helicopter model.
%
% Input trajectories must be at least C^2 continuous (position, velocity, acceleration).
% The mapping assumes the simplified rigid-body model described in Eq. (1).
classdef FlatnessMap
    
    methods (Static)
        %% MAP Algebraic transformation from flat outputs to reference states and inputs.
        % 
        % Args:
        %   z   - Flat output vector [xI; yI; zI; Psi] in Inertial Frame.
        %   dz  - First derivative of flat outputs (Velocities).
        %   ddz - Second derivative of flat outputs (Accelerations).
        %
        % Returns:
        %   xref - Reconstructed state vector [10x1] (Eq. 4).
        %   uref - Reconstructed feedforward input vector [4x1] (Eq. 5-8).
        %
        % Preconditions:
        %   size(z) == [4,1], size(dz) == [4,1], size(ddz) == [4,1].
        % Postconditions:
        %   xref(9:10) == 0 (Integral states are initialized to zero reference).
        function [xref, uref] = map(z, dz, ddz)
            
            mp = ModelParams;
            
            % UNPACK FLAT OUTPUTS 
            Psi   = z(4);
            dPsi  = dz(4);
            ddPsi = ddz(4);
            
            dxI   = dz(1); 
            dyI   = dz(2);
            ddxI  = ddz(1); 
            ddyI  = ddz(2);
            
            % STATE RECONSTRUCTION  
            % Transform inertial velocities to body frame: v_B = R(Psi)^T * v_I
            sin_p = sin(Psi);
            cos_p = cos(Psi);
            
            dxB =  cos_p * dxI + sin_p * dyI;
            dyB = -sin_p * dxI + cos_p * dyI;
            dzB = dz(3); 
            
            % Assemble Reference State Vector [10x1]
            % [x, y, z, Psi, dxB, dyB, dzB, dPsi, xi, yi]
            xref = [z(1); z(2); z(3); Psi; ...   
                    dxB; dyB; dzB; dPsi; ...     
                    0; 0];                       
            
            % INPUT RECONSTRUCTION  
            % Kinematic Differentiation: Projection of Inertial Accel to Body Frame
            % a_B_kin = R^T * a_I + d/dt(R^T) * v_I
            term_proj_x =  cos_p * ddxI + sin_p * ddyI;
            term_proj_y = -sin_p * ddxI + cos_p * ddyI;
            
            % Required Physical Accelerations (Derivations including Coriolis terms)
            ddxB_req = term_proj_x + dPsi * dyB; 
            ddyB_req = term_proj_y - dPsi * dxB;
            ddzB_req = ddz(3);
            
            % Dynamics Inversion: Solve for u given requested accelerations
            % Note: Kinematic Coriolis terms effectively cancel dynamic coupling 
            % in the simplified model formulation.
            
            % Pitch control input
            ux = (1 / mp.bx) * (ddxB_req - mp.kx * dxB - dPsi * dyB);
            
            % Roll control input
            uy = (1 / mp.by) * (ddyB_req - mp.ky * dyB + dPsi * dxB);
            
            % Normalized vertical thrust
            uz = (1 / mp.bz) * (ddzB_req + mp.g);
            
            % Yaw control torque
            uPsi = (1 / mp.bPsi) * (ddPsi - mp.kPsi * dPsi);
            
            % Assemble Reference Input Vector [4x1]
            uref = [ux; uy; uz; uPsi];
        end
    end
end