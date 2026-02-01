% MPCCONTROLLER Linear Time-Varying Model Predictive Control 
% Implements the LTV-MPC architecture with a Warm-Start strategy for the 
% miniature coaxial helicopter, as described in Kunz et al. (2013).
%
% Modified version to account different Ts.

classdef MPCControllerv2 < handle
    
    properties
        model       % HelicopterModel
        cp          % ControlParams
        Q, R, Qf    % Weights
        nominal_x, nominal_u
        qp_options
    end
    
    methods
        %% CONSTRUCTOR (Updated for Ts)
        function obj = MPCControllerv2(model_obj, cp_override)
            
            obj.model = model_obj;
            
            % Load Parameters 
            if nargin > 1 && ~isempty(cp_override)
                obj.cp = cp_override;
            else
                obj.cp = ControlParams; 
            end
            
            % Setup Weights
            obj.Q = diag(obj.cp.Q_diag);
            obj.R = diag(obj.cp.R_diag);
            
            % Terminal Cost 
            if obj.cp.use_computed_terminal_cost
                obj.Qf = TerminalCost.compute(obj.Q, obj.R, obj.model, obj.cp.Ts);
            else
                obj.Qf = obj.Q; 
            end
            
            % Solver Options
            obj.qp_options = optimoptions('quadprog', 'Display', 'off', ...
                'Algorithm', 'interior-point-convex'); 
        end
        
        %% INIT: Cold Start Strategy
        function init(obj, x0, trajectory_obj, t0)
            N = obj.cp.p;
            nx = 10; nu = 4;
            Ts = obj.cp.Ts;
            
            obj.nominal_x = zeros(nx, N+1);
            obj.nominal_u = zeros(nu, N);
            
            % Populate nominal trajectory from Flatness
            for k = 0:N
                t_k = t0 + k * Ts;
                [z, dz, ddz] = trajectory_obj.get_flat_outputs(t_k);
                [xref, uref] = FlatnessMap.map(z, dz, ddz);
                obj.nominal_x(:, k+1) = xref;
                if k < N
                    obj.nominal_u(:, k+1) = uref;
                end
            end
            obj.nominal_x(:, 1) = x0;
        end
        
        %% SOLVE: Optimization Step
        function [u_next, debug_info] = solve(obj, x_meas, xref_seq, uref_seq)
            
            N  = obj.cp.p;
            Ts = obj.cp.Ts;
            nx = 10; nu = 4;
            
            obj.nominal_x(:, 1) = x_meas;
            
            % Sequential Linearization (Euler based on paper)
            Ad_seq = cell(1, N);
            Bd_seq = cell(1, N);
            d_seq  = cell(1, N);
            
            x_curr_sim = x_meas;
            
            for k = 1:N
                u_lin = obj.nominal_u(:, k);
                x_lin = x_curr_sim; 
                
                [Ac, Bc] = obj.model.get_jacobians(x_lin, u_lin);
                [Ad, Bd] = Discretization.discretize(Ac, Bc, Ts, 'euler'); 
                
                % Affine correction
                dxdt = obj.model.dynamics(0, x_lin, u_lin); 
                x_next_nl = x_lin + dxdt * Ts;
                d_val = x_next_nl - (Ad * x_lin + Bd * u_lin);
                
                Ad_seq{k} = Ad;
                Bd_seq{k} = Bd;
                d_seq{k}  = d_val;
                
                x_curr_sim = x_next_nl; 
                obj.nominal_x(:, k+1) = x_next_nl;
            end
            
            % Sparse QP Assembly
            u_lims.min = obj.model.mp.u_min;
            u_lims.max = obj.model.mp.u_max;
            
            % Relaxed state limits
            x_lims.min = [-inf; -inf; -inf; -inf; -obj.model.mp.v_max; -obj.model.mp.psi_rate_max; -inf; -inf];
            x_lims.max = [ inf;  inf;  inf;  inf;  obj.model.mp.v_max;  obj.model.mp.psi_rate_max;  inf;  inf];
            
            [H, f, Aeq, beq, lb, ub] = QPBuilder.build(...
                Ad_seq, Bd_seq, d_seq, ...
                x_meas, xref_seq, uref_seq, ...
                obj.Q, obj.R, obj.Qf, u_lims, x_lims);
                
            [z_opt, ~, exitflag] = quadprog(H, f, [], [], Aeq, beq, lb, ub, [], obj.qp_options);
            
            if exitflag < 0
                warning('MPC:QP_Fail', 'Solver failure (Flag %d).', exitflag);
                u_next = obj.nominal_u(:, 1);
                debug_info.cost = NaN;
                return;
            end
            
            % Extract result
            idx_u0 = nx + 1;
            u_next = z_opt(idx_u0 : idx_u0+nu-1);
            
            % Warm start update
            u_opt_seq = zeros(nu, N);
            for k = 1:N
                idx_u = (k-1)*(nx+nu) + nx + 1;
                u_opt_seq(:, k) = z_opt(idx_u : idx_u+nu-1);
            end
            obj.nominal_u = [u_opt_seq(:, 2:end), u_opt_seq(:, end)];
            
            debug_info.cost = 0; 
        end
    end
end