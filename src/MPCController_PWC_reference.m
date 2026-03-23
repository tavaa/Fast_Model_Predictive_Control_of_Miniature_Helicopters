% MPCCONTROLLER Linear Time-Varying Model Predictive Control 
% Implements the LTV-MPC architecture with a Warm-Start strategy for a 
% miniature coaxial helicopter. Handles receding horizon optimization by 
% performing sequential linearization, constructing a sparse QP, and 
% invoking a convex solver.
%
% PWC version (works with PWC reference trajectory generated in
% models/GeneratePWC_reference.m

classdef MPCController_PWC_reference < handle
    
    properties
        model       
        cp          
        
        Q           % [nx x nx] State error penalty matrix
        R           % [nu x nu] Input penalty matrix
        Qf          % [nx x nx] Terminal state penalty matrix
        
        nominal_x   % [nx x N+1] Nominal state trajectory for linearization
        nominal_u   % [nu x N] Nominal input trajectory for linearization
        
        qp_options  % Solver configuration for the convex optimizer
    end
    
    methods
        
        % CONSTRUCTOR
        % Initializes controller weights, terminal costs, and solver settings.
        %
        % INPUTS:
        %   model_obj - Instance of HelicopterModel defining system dynamics
        function obj = MPCController_PWC_reference(model_obj)
            obj.model = model_obj;
            obj.cp = ControlParams; 
            
            obj.Q = diag(obj.cp.Q_diag);
            obj.R = diag(obj.cp.R_diag);
            
            % Compute terminal cost via DARE or default to stage cost
            if obj.cp.use_computed_terminal_cost
                obj.Qf = TerminalCost.compute(obj.Q, obj.R, obj.model, obj.cp.Ts);
            else
                obj.Qf = obj.Q; 
            end
            
            obj.qp_options = optimoptions('quadprog', 'Display', 'off', ...
                'Algorithm', 'interior-point-convex'); 
        end
        
        % init
        % Generates the initial nominal trajectory (Cold Start) by sampling 
        % the provided reference trajectory.
        %
        % INPUTS:
        %   x0    - [10x1] Initial measured state
        %   x_ref - [10xT] Reference state trajectory
        %   u_ref - [4xT]  Reference input trajectory
        function init(obj, x0, x_ref, u_ref)
            N  = obj.cp.p;
            nx = 10;
            nu = 4;

            % Total available reference length
            T_ref = size(x_ref, 2);

            % Preallocate nominal trajectories
            obj.nominal_x = zeros(nx, N+1);
            obj.nominal_u = zeros(nu, N);

            % Initial population using available reference
            for k = 1:N+1
                idx = min(k, T_ref); % Prevent out-of-bounds
                obj.nominal_x(:, k) = x_ref(:, idx);
            end

            for k = 1:N
                idx = min(k, size(u_ref, 2));
                obj.nominal_u(:, k) = u_ref(:, idx);
            end

            % Synchronize first state with actual measurement
            obj.nominal_x(:, 1) = x0;
        end
        

        % METHOD: solve
        % Executes the Receding Horizon Optimization step.
        %
        % INPUTS:
        %   x_meas   - [10x1]     Current state feedback measurement
        %   xref_seq - [10x(N+1)] Reference state trajectory over horizon
        %   uref_seq - [4xN]      Reference input trajectory over horizon
        %
        % OUTPUTS:
        %   u_next     - [4x1]  Optimal control input 
        %   debug_info 
        function [u_next, debug_info] = solve(obj, x_meas, xref_seq, uref_seq)
            N  = obj.cp.p;
            Ts = obj.cp.Ts;
            nx = 10; 
            nu = 4;
            
            % Update operating point 
            obj.nominal_x(:, 1) = x_meas;
            
            % Sequential LTV Linearization Loop
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
                
                % Affine correction term: d_k = f(x,u) - (Ad*x + Bd*u)

                % v1: CORRECTION HERE !!!!
                %dxdt = obj.model.dynamics(0, x_lin, u_lin); 
                %x_next_nl = x_lin + dxdt * Ts;
                %d_val = x_next_nl - (Ad * x_lin + Bd * u_lin);
                
                % v2: major precision inside MPC controller
                [~, x_sim_temp] = ode45(@(t,x) obj.model.dynamics(t, x, u_lin), [0 Ts], x_lin);
                x_next_nl = x_sim_temp(end, :)';

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
            
            % State validity constraints (Velocity and Yaw-rate bounds)
            v_max = obj.model.mp.v_max; 
            r_max = obj.model.mp.psi_rate_max;
            x_lims.min = [-inf; -inf; -inf; -inf; -v_max; -r_max; -inf; -inf];
            x_lims.max = [ inf;  inf;  inf;  inf;  v_max;  r_max;  inf;  inf];
            
            % Invoke Matrix Factory for sparse QP formulation
            [H, f, Aeq, beq, lb, ub] = QPBuilder.build(...
                Ad_seq, Bd_seq, d_seq, ...
                x_meas, xref_seq, uref_seq, ...
                obj.Q, obj.R, obj.Qf, u_lims, x_lims);
                
            % Numerical Optimization
            [z_opt, ~, exitflag] = quadprog(H, f, [], [], Aeq, beq, lb, ub, [], obj.qp_options);
            
            % Exception Handling: Fallback to nominal input on solver failure
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
            
            % Explicit computation of the quadratic tracking cost
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
            obj.nominal_u = [u_opt_seq(:, 2:end), u_opt_seq(:, end)];
            obj.nominal_x = [x_opt_seq(:, 2:end), x_opt_seq(:, end)];
            
            % Diagnostic Serialization
            debug_info.x_opt = x_opt_seq;
            debug_info.u_opt = u_opt_seq;
            debug_info.cost  = J_real; 
        end
    end
end