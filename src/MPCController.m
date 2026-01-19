% MPCCONTROLLER Linear Time-Varying Model Predictive Control 
% Implements the LTV-MPC architecture with a Warm-Start strategy for the 
% miniature coaxial helicopter, as described in Kunz et al. (2013).
%
% This class handles the receding horizon optimization by performing 
% sequential linearization of the nonlinear dynamics along a nominal path, 
% constructing a sparse Quadratic Program (QP), and invoking a convex solver 
% at each sampling interval.
%
% Control Logic Flow:
%   1. Measurement Acquisition: Capture the current estimated state.
%   2. Sequential Linearization: Compute LTV matrices (Ad, Bd, d) along the nominal trajectory.
%   3. Sparse QP Formulation: Assemble the Hessian, linear cost, and equality constraints.
%   4. Numerical Optimization: Solve the resulting QP for the optimal control sequence.
%   5. Actuation: Apply the first optimal control input to the plant.
%   6. Warm Start Shift: Update the nominal trajectory for the subsequent time step.

classdef MPCController < handle
    
    properties
        model       % HelicopterModel: Predictive nonlinear plant model
        cp          % ControlParams: MPC hyper-parameters and weighting factors
        
        % Objective Function Weighting Matrices (Positive Semi-Definite)
        Q           % State error penalty matrix [nx x nx]
        R           % Input error penalty matrix [nu x nu]
        Qf          % Terminal state penalty matrix [nx x nx]
        
        % Nominal Trajectories (Operating points for LTV linearization)
        % nominal_x [nx x N+1], nominal_u [nu x N].
        nominal_x
        nominal_u
        
        % Solver configuration for the convex optimizer
        qp_options
    end
    
    methods
        %% CONSTRUCTOR: Initializes controller weights and solver settings.
        %
        % Args:
        %   model_obj - Instance of HelicopterModel defining system dynamics.
        function obj = MPCController(model_obj)
            % Initialize system components
            obj.model = model_obj;
            obj.cp = ControlParams; 
            
            % Objective Function Weighting Setup
            obj.Q = diag(obj.cp.Q_diag);
            obj.R = diag(obj.cp.R_diag);
            
            % Terminal Cost Calculation:
            % Implements either a DARE-based solution for local asymptotic stability 
            % or falls back to the paper's default stage-cost-based terminal penalty.
            if obj.cp.use_computed_terminal_cost
                % Compute via Discrete Algebraic Riccati Equation (DARE)
                obj.Qf = TerminalCost.compute(obj.Q, obj.R, obj.model, obj.cp.Ts);
            else
                % Default: Qf = Q.
                obj.Qf = obj.Q; 
            end
            
            % Solver Configuration
            % Algorithm: interior-point-convex, optimized for sparse quadratic objectives.
            obj.qp_options = optimoptions('quadprog', 'Display', 'off', ...
                'Algorithm', 'interior-point-convex'); 
        end
        
        %% INIT: Executes the "Cold Start" initialization strategy.
        %
        % Generates the initial nominal trajectory by sampling the 
        % kinematically feasible path provided by the differential flatness mapping.
        %
        % Args:
        %   x0             - Measured initial state.
        %   trajectory_obj - Instance of TrajectoryBase subclass.
        %   t0             - Simulation start time.
        function init(obj, x0, trajectory_obj, t0)
            
            N = obj.cp.p;
            nx = 10; nu = 4;
            Ts = obj.cp.Ts;
            
            obj.nominal_x = zeros(nx, N+1);
            obj.nominal_u = zeros(nu, N);
            
            % Populate nominal trajectory over the prediction horizon
            for k = 0:N
                t_k = t0 + k * Ts;
                
                % Algebraic Flatness Mapping: Flat Outputs -> Physical States/Inputs
                [z, dz, ddz] = trajectory_obj.get_flat_outputs(t_k);
                [xref, uref] = FlatnessMap.map(z, dz, ddz);
                
                obj.nominal_x(:, k+1) = xref;
                if k < N
                    obj.nominal_u(:, k+1) = uref;
                end
            end
            
            % Synchronize the first nominal state with the actual initial condition
            obj.nominal_x(:, 1) = x0;
        end
        
        %% SOLVE: Executes the Receding Horizon Optimization step.
        %
        % Args:
        %   x_meas   - Latest state feedback measurement [10x1].
        %   xref_seq - Reference state trajectory over the horizon [10xN+1].
        %   uref_seq - Reference input trajectory over the horizon [4xN].
        %
        % Returns:
        %   u_next     - Optimal control action to be applied to the plant.
        %   debug_info - Diagnostic telemetry (optimal sequences and cost).
        function [u_next, debug_info] = solve(obj, x_meas, xref_seq, uref_seq)
            
            N  = obj.cp.p;
            Ts = obj.cp.Ts;
            nx = 10; nu = 4;
            
            % Update Operating Point 
            obj.nominal_x(:, 1) = x_meas;
            
            % Sequential LTV Linearization Loop 
            % Constructs the time-varying linear approximation along the nominal path.
            Ad_seq = cell(1, N);
            Bd_seq = cell(1, N);
            d_seq  = cell(1, N);
            
            x_curr_sim = x_meas;
            
            for k = 1:N
                u_lin = obj.nominal_u(:, k);
                x_lin = x_curr_sim; 
                
                % Compute Jacobians and discretize via Euler integration
                [Ac, Bc] = obj.model.get_jacobians(x_lin, u_lin);
                [Ad, Bd] = Discretization.discretize(Ac, Bc, Ts, 'euler'); 
                
                % Affine Correction Term: d_k = f(x,u) - (Ad*x + Bd*u)
                % Compensates for linearization errors to ensure prediction fidelity.
                dxdt = obj.model.dynamics(0, x_lin, u_lin); 
                x_next_nl = x_lin + dxdt * Ts;
                d_val = x_next_nl - (Ad * x_lin + Bd * u_lin);
                
                Ad_seq{k} = Ad;
                Bd_seq{k} = Bd;
                d_seq{k}  = d_val;
                
                % Propagate nominal state for the next step in the horizon
                x_curr_sim = x_next_nl; 
                obj.nominal_x(:, k+1) = x_next_nl;
            end
            
            % Sparse QP Problem Assembly 
            u_lims.min = obj.model.mp.u_min;
            u_lims.max = obj.model.mp.u_max;
            
            % State Validity Constraints (Velocity and Yaw-rate bounds)
            v_max = obj.model.mp.v_max; 
            r_max = obj.model.mp.psi_rate_max;
            x_lims.min = [-inf; -inf; -inf; -inf; -v_max; -r_max; -inf; -inf];
            x_lims.max = [ inf;  inf;  inf;  inf;  v_max;  r_max;  inf;  inf];
            
            % Invoke the Matrix Factory for sparse QP formulation
            [H, f, Aeq, beq, lb, ub] = QPBuilder.build(...
                Ad_seq, Bd_seq, d_seq, ...
                x_meas, xref_seq, uref_seq, ...
                obj.Q, obj.R, obj.Qf, u_lims, x_lims);
                
            % Numerical Optimization
            % Solves for the stacked decision vector Z = [x0, u0, ..., xN]
            [z_opt, ~, exitflag] = quadprog(H, f, [], [], Aeq, beq, lb, ub, [], obj.qp_options);
            
            % Exception Handling: Fallback to last known nominal input on solver failure
            if exitflag < 0
                warning('MPC:QP_Fail', 'Solver failure (Flag %d). Reverting to feedforward input.', exitflag);
                u_next = obj.nominal_u(:, 1);
                
                debug_info.x_opt = obj.nominal_x;
                debug_info.u_opt = obj.nominal_u;
                debug_info.cost  = NaN; 
                return;
            end
            
            % Trajectory De-stacking & Solution Extraction 
            idx_u0 = nx + 1;
            u_next = z_opt(idx_u0 : idx_u0+nu-1);
            
            x_opt_seq = zeros(nx, N+1);
            u_opt_seq = zeros(nu, N);
            
            for k = 1:N
                idx_x = (k-1)*(nx+nu) + 1;
                idx_u = idx_x + nx;
                
                x_opt_seq(:, k) = z_opt(idx_x : idx_x+nx-1);
                u_opt_seq(:, k) = z_opt(idx_u : idx_u+nu-1);
            end
            x_opt_seq(:, N+1) = z_opt(end-nx+1 : end);
            
            % Objective Function (J)
            % Explicit computation of the quadratic tracking cost for diagnostics.
            J_real = 0;
            
            % Accumulate Stage Cost
            for k = 1:N
                ex = x_opt_seq(:, k) - xref_seq(:, k);
                eu = u_opt_seq(:, k) - uref_seq(:, k);
                J_real = J_real + (ex' * obj.Q * ex) + (eu' * obj.R * eu);
            end
            
            % Add Terminal Cost
            ex_N = x_opt_seq(:, N+1) - xref_seq(:, N+1);
            J_real = J_real + (ex_N' * obj.Qf * ex_N);
            
            % Warm Start Shift Strategy 
            % Propagate the optimal sequence forward to provide the operating 
            % point for the next LTV linearization cycle.
            obj.nominal_u = [u_opt_seq(:, 2:end), u_opt_seq(:, end)];
            obj.nominal_x = [x_opt_seq(:, 2:end), x_opt_seq(:, end)];
            
            % Diagnostic Serialization
            debug_info.x_opt = x_opt_seq;
            debug_info.u_opt = u_opt_seq;
            debug_info.cost  = J_real; 
        end
    end
end