% TERMINALCOST Discrete Linear Quadratic Regulator (DLQR) Solver
%
%   Computes the terminal weight matrix Qf to ensure local asymptotic 
%   stability of the LTV-MPC scheme.
%
%   The terminal cost Qf is identified as the infinite-horizon cost-to-go 
%   matrix P obtained from a DLQR design. This implementation utilizes the 
%   'dlqr' command to solve the underlying Riccati Equation and validates 
%   that the resulting closed-loop (CL) system (Ad - Bd*K) is Asymptotically 
%   Stable (A.S.) by inspecting the radius of the CL eigenvalues.

classdef TerminalCost
    methods (Static)
        %% COMPUTE Calculates and verifies the optimal Qf matrix via DLQR.
        %
        % Args:
        %   Q     - [nx x nx] State weighting matrix (PSD).
        %   R     - [nu x nu] Input weighting matrix (PD).
        %   model - HelicopterModel instance for Jacobian evaluation.
        %   Ts    - Scalar sampling time [s].
        %   x, u  - (Optional) Terminal reference point for linearization.
        %
        % Returns:
        %   Qf    - [nx x nx] The steady-state cost-to-go matrix P.
        %
        % Preconditions:
        %   - The pair (Ad, Bd) must be stabilizable.
        %   - R must be strictly positive definite (R > 0).
        %
        % Postconditions:
        %   - Qf is the unique positive-definite solution to the DARE.
        %   - All CL eigenvalues 'e' satisfy |e| < 1.
        function Qf = compute(Q, R, model, Ts, x, u)
            
            % If no linearization point is provided, assume Hover equilibrium.
            if nargin < 5
                x = zeros(10, 1); x(3) = 1.0; 
                u = [0; 0; model.mp.g / model.mp.bz; 0];
            end
            
            % Linearization (Jacobians)
            [Ac, Bc] = model.get_jacobians(x, u);
            
            % Discretization
            % Transform continuous dynamics into discrete-time (Ad, Bd) using ZOH.
            nx = size(Ac, 1);
            nu = size(Bc, 2);
            sys_c = ss(Ac, Bc, eye(nx), zeros(nx, nu));
            sys_d = c2d(sys_c, Ts);
            Ad = sys_d.A;
            Bd = sys_d.B;
            
            % DLQR Synthesis
            % K: Optimal Gain, P: Cost-to-go (Qf), e: CL Eigenvalues
            try
                [K, P, e] = dlqr(Ad, Bd, Q, R);
            catch ME
                warning('MPC:TerminalCost:DLQR_Fail', 'DLQR synthesis failed: %s. Using Q.', ME.message);
                Qf = Q;
                return;
            end
            
            % Stability Verification 
            % According to Lyapunov stability for discrete-time systems, 
            % the max absolute eigenvalue must be < 1.
            spectral_radius = max(abs(e));
            
            if spectral_radius >= 1.0
                warning('MPC:TerminalCost:Unstable', ...
                    'The calculated DLQR is NOT Asymptotically Stable. Spectral Radius: %.4f', ...
                    spectral_radius);
            else
                % Asymptotic Stability verified: the terminal cost is a valid Lyapunov candidate.
            end
            
            % Assign the steady-state cost matrix P as the MPC Terminal Cost
            Qf = P;
        end
    end
end