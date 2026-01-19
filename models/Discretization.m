classdef Discretization
    % DISCRETIZATION Static utility for converting Continuous -> Discrete models.
    %
    % Methods:
    %   - 'euler': Forward Euler (Used in the Paper).
    %              Ad = I + A*Ts, Bd = B*Ts.
    %              Fast but inaccurate for large Ts or stiff systems.
    %
    %   - 'zoh':   Zero-Order Hold (Exact solution for constant inter-sample input).
    %              Uses Matrix Exponential.
    %              Ad = expm(A*Ts), Bd = integral(expm(A*t))*B.
    
    methods (Static)
        function [Ad, Bd] = discretize(A, B, Ts, method)
            % DISCRETIZE Converts continuous (A,B) to discrete (Ad,Bd).
            %
            % Args:
            %   A, B: Continuous system matrices
            %   Ts:   Sampling time [s]
            %   method: 'euler' or 'zoh'
            
            [nx, nu] = size(B);
            
            if nargin < 4
                method = 'euler'; % Default to Paper method
            end
            
            switch lower(method)
                case 'euler'
                    % Paper Implementation 
                    % Approx: dx/dt ~ (x_k+1 - x_k)/Ts
                    Ad = eye(nx) + A * Ts;
                    Bd = B * Ts;
                    
                case 'zoh'
                    % Exact Discretization (Matrix Exponential)
                    % Construct big matrix M = [A B; 0 0] * Ts
                    M = [A, B; zeros(nu, nx), zeros(nu, nu)] * Ts;
                    
                    % Compute matrix exponential
                    EM = expm(M);
                    
                    % Extract Ad, Bd
                    % EM = [Ad, Bd; 0, I]
                    Ad = EM(1:nx, 1:nx);
                    Bd = EM(1:nx, nx+1:nx+nu);
                    
                otherwise
                    error('Unknown discretization method: %s. Use "euler" or "zoh".', method);
            end
        end
    end
end