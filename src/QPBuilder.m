% QPBUILDER Sparse Quadratic Programming Matrix Factory
% This class provides a static interface for constructing the sparse matrices 
% required to solve the LTV-MPC optimization problem. It transforms the 
% optimal control problem into a standard QP form:
%
%   minimize   0.5 * Z' * H * Z + f' * Z
%   subject to Aeq * Z = beq
%              lb <= Z <= ub
%
% The decision variable vector is organized as:
%   Z = [x_0', u_0', x_1', u_1', ..., u_{N-1}', x_N']'
%
% This choice provides:
% - Efficiency: Matrices are constructed in SPARSE form to minimize memory 
%   footprint and optimize solver performance (Interior-Point algorithms).
% - Consistency: Dynamics are enforced as equality constraints using the 
%   affine linearized form: -Ad*x_k - Bd*u_k + I*x_{k+1} = d_k.

classdef QPBuilder
    methods (Static)
        
        %% BUILD Constructs the standard QP problem matrices.
        %
        % Args:
        %   Ad_seq, Bd_seq, d_seq - Cell arrays [1xN] of LTV system matrices.
        %   x0_meas      - Current state measurement [nx x 1].
        %   xref_seq     - State reference trajectory [nx x N+1].
        %   uref_seq     - Input feedforward trajectory [nu x N].
        %   Q, R, Qf     - Weighting matrices (Diagonal).
        %   u_lims, x_lims - Structs containing .min and .max fields.
        %
        % Returns:
        %   H   - Sparse Hessian matrix (Positive Semi-Definite).
        %   f   - Linear cost vector.
        %   Aeq - Sparse equality constraint matrix (Block-Banded).
        %   beq - RHS vector for equality constraints (Initial state + Affine terms).
        %   lb, ub - Lower and upper bound vectors for decision variables.
        %
        % Preconditions:
        %   - length(Ad_seq) == N (Prediction Horizon).
        %   - Q, R, Qf must be diagonal or positive semi-definite.
        %
        % Postconditions:
        %   - size(Z) = (N+1)*nx + N*nu.
        %   - Aeq is a sparse matrix with a specific banded structure for KKT efficiency.
        function [H, f, Aeq, beq, lb, ub] = build(Ad_seq, Bd_seq, d_seq, ...
                                                  x0_meas, xref_seq, uref_seq, ...
                                                  Q, R, Qf, u_lims, x_lims)
            
            % PROBLEM DIMENSIONS 
            N  = length(Ad_seq);
            nx = size(Ad_seq{1}, 1);
            nu = size(Bd_seq{1}, 2);
            nz = (N+1)*nx + N*nu; % Total decision variables
            
            %% COST FUNCTION CONSTRUCTION (H, f)
            % H is a block-diagonal matrix of weights. f is a vector mapping
            % the reference trajectory into the quadratic objective.
            
            H_blocks = cell(1, 2*N + 1);
            f_cell   = cell(1, 2*N + 1);
            
            % Enforce sparsity on weight blocks for memory efficiency
            Q_sp  = sparse(Q);
            R_sp  = sparse(R);
            Qf_sp = sparse(Qf);
            
            for k = 1:N
                H_blocks{2*k - 1} = Q_sp;
                H_blocks{2*k}     = R_sp;
                
                % Linear gradient term: f = -Weight * Reference
                f_cell{2*k - 1} = -Q * xref_seq(:, k);
                f_cell{2*k}     = -R * uref_seq(:, k);
            end
            
            % Terminal block (N+1)
            H_blocks{2*N + 1} = Qf_sp;
            f_cell{2*N + 1}   = -Qf * xref_seq(:, N+1);
            
            H = blkdiag(H_blocks{:});
            f = vertcat(f_cell{:});
            
            %% EQUALITY CONSTRAINTS CONSTRUCTION (Aeq, beq)
            % Maps initial conditions and linearized dynamics.
            
            num_eq_con = (N+1) * nx;
            
            % Pre-allocate sparse Aeq based on estimated non-zero (nnz) count
            % Structure: N blocks of [-A, -B, I] + initial condition I.
            nz_est = num_eq_con * (nx + nu + 1); 
            Aeq = spalloc(num_eq_con, nz, nz_est);
            beq = zeros(num_eq_con, 1);
            
            % Constraint 1: Initial Condition Reconstruction 
            % x_0 = x_meas => [I 0 0 ...] * Z = x_meas
            Aeq(1:nx, 1:nx) = speye(nx); 
            beq(1:nx)       = x0_meas;
            
            % Constraint 2: LTV Dynamics Propagation 
            % -Ad_k * x_k - Bd_k * u_k + I * x_{k+1} = d_k
            row_idx = nx + 1;
            
            for k = 1:N
                % Compute indices within the global decision vector Z
                idx_xk   = (k-1)*(nx+nu) + 1;
                idx_uk   = idx_xk + nx;
                idx_xkp1 = idx_uk + nu;
                
                rows = row_idx : row_idx+nx-1;
                
                % Dynamic Coefficients (Sparse insertion)
                Aeq(rows, idx_xk : idx_xk+nx-1)     = sparse(-Ad_seq{k});
                Aeq(rows, idx_uk : idx_uk+nu-1)     = sparse(-Bd_seq{k});
                Aeq(rows, idx_xkp1 : idx_xkp1+nx-1) = speye(nx);
                
                % Right-Hand Side: Includes the affine linearization term d_k
                beq(rows) = d_seq{k};
                
                row_idx = row_idx + nx;
            end
            
            %% INEQUALITY CONSTRAINTS (lb, ub)
            % Hard constraints on states (validity region) and inputs (actuators).
            
            lb = -inf(nz, 1);
            ub =  inf(nz, 1);
            
            for k = 1:N
                idx_xk = (k-1)*(nx+nu) + 1;
                idx_uk = idx_xk + nx;
                
                % Apply state bounds if defined (e.g., velocity limits)
                if ~isempty(x_lims)
                    lb(idx_xk : idx_xk+nx-1) = x_lims.min;
                    ub(idx_xk : idx_xk+nx-1) = x_lims.max;
                end
                
                % Apply normalized actuator limits u_min, u_max
                lb(idx_uk : idx_uk+nu-1) = u_lims.min;
                ub(idx_uk : idx_uk+nu-1) = u_lims.max;
            end
            
            % Terminal state boundary constraints
            idx_xN = N*(nx+nu) + 1;
            if ~isempty(x_lims)
                lb(idx_xN : idx_xN+nx-1) = x_lims.min;
                ub(idx_xN : idx_xN+nx-1) = x_lims.max;
            end
        end
    end
end