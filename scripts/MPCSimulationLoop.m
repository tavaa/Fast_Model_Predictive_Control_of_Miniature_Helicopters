% SIMULATION ORCHESTRATOR: LTV-MPC FOR MINIATURE HELICOPTERS
%
% Evaluates the performance of a Receding Horizon controller across 
% diverse flight regimes (Regulation and Trajectory Tracking).
%
% Pipeline:
%   1. Environmental Setup: Register paths and initialize versioned results.
%   2. System Instantiation: Load physical parameters and model objects.
%   3. Stability Configuration: Apply DARE-based terminal costs if enabled.
%   4. Closed-Loop Execution Loop:
%      - Sampling: Discretize reference trajectory over horizon p.
%      - Adaptive Qf: Update terminal weight based on terminal reference.
%      - Optimization: Solve Sparse QP via LTV linearization.
%      - Integration: Propagate nonlinear plant dynamics via ODE45.
%   5. Data Serialization: Export CSV and simulation config JSON.

clear; clc; close all;

%% ENVIRONMENTAL SETUP & DIRECTORY VERSIONING
[scriptDir, ~, ~] = fileparts(mfilename('fullpath'));
projectRoot = fileparts(scriptDir);

resultsBaseDir = fullfile(projectRoot, 'results/MPC');
if ~exist(resultsBaseDir, 'dir'), mkdir(resultsBaseDir); end

todayStr = datestr(now, 'yyyy-mm-dd');
runID = 1;
while true
    runDirName = sprintf('%s_Run_%02d', todayStr, runID);
    fullRunDir = fullfile(resultsBaseDir, runDirName);
    if ~exist(fullRunDir, 'dir')
        mkdir(fullRunDir);
        break;
    end
    runID = runID + 1;
end

fprintf('      MINIATURE HELICOPTER MPC SIMULATION          \n');
fprintf('[SYSTEM] Output Directory: %s\n', runDirName);

%% SYSTEM INITIALIZATION
% Parameter instantiation
mp = ModelParams;
cp = ControlParams;
tp = TrajectoryParams;

% Global Weighting Matrices
Q_mat = diag(cp.Q_diag);
R_mat = diag(cp.R_diag);

% Plant Dynamics Model (Full Physics)
real_helicopter = HelicopterModel(false); 

% Predictive Controller (LTV Model)
mpc = MPCController(HelicopterModel(false));

% Define Flight Scenarios: {Identifier, TrajectoryObject, Duration}
scenarios = {
    'Hover',  ShapeHover(tp.Hover),   tp.T_duration_hover;
    'Circle', ShapeCircle(tp.Circle), tp.T_duration_circle;
    'Spiral', ShapeSpiral(tp.Spiral), tp.T_duration_spiral
};

% Data structure for statistical summary
metrics_data = {};

%% EXPORT JSON
% Captures current hyper-parameters to ensure research reproducibility.
configStruct.ModelParams = struct(...
    'bx', mp.bx, 'by', mp.by, 'bz', mp.bz, ...
    'kx', mp.kx, 'ky', mp.ky, ...
    'u_min', mp.u_min, 'u_max', mp.u_max);

configStruct.ControlParams = struct(...
    'Ts', cp.Ts, ...
    'Horizon_p', cp.p, ...
    'Q_diag', cp.Q_diag, ...
    'R_diag', cp.R_diag, ...
    'Stability_Enhanced_Qf', cp.use_computed_terminal_cost);

configStruct.TrajectoryParams = struct(...
    'Hover', tp.Hover, ...
    'Circle', tp.Circle, ...
    'Spiral', tp.Spiral);

jsonStr = jsonencode(configStruct, 'PrettyPrint', true);
fid = fopen(fullfile(fullRunDir, 'simulation_config.json'), 'w');
if fid == -1, error('Failed to initialize JSON config'); end
fwrite(fid, jsonStr, 'char');
fclose(fid);
fprintf('[SYSTEM] Global system configuration exported to JSON.\n');

%% SIMULATION PIPELINE
for s = 1:size(scenarios, 1)
    
    scenario_name = scenarios{s, 1};
    traj_obj      = scenarios{s, 2};
    T_end         = scenarios{s, 3};
    
    fprintf('\n[SCENARIO] Starting: %s (Duration: %.1fs)\n', scenario_name, T_end);
    
    % Initial Cold Start
    [z0, dz0, ddz0] = traj_obj.get_flat_outputs(0);
    [x0, ~] = FlatnessMap.map(z0, dz0, ddz0);
    
    current_x = x0(:);
    mpc.init(current_x, traj_obj, 0);
    
    % Static Terminal Cost Setup 
    % For regulation tasks (Hover), Qf is computed once at the start 
    % at the equilibrium point to minimize computational overhead.
    if cp.use_computed_terminal_cost && strcmp(scenario_name, 'Hover')
        u_eq = [0; 0; mp.g/mp.bz; 0]; % Equilibrium gravity compensation
        mpc.Qf = TerminalCost.compute(Q_mat, R_mat, mpc.model, cp.Ts, x0, u_eq);
        fprintf('    [INFO] Steady-state Qf initialized for Hover.\n');
    end
    
    % Temporal discretization
    t_steps = 0:cp.Ts:T_end;
    N_steps = length(t_steps);
    
    % Preallocate buffer (20 Channels)
    res_data = zeros(N_steps, 20);
    
    % Statistical Counters
    sum_sq_err_x = 0;   
    sum_calc_time = 0;  
    
    fprintf('    Simulating... ');
    
    % Iterative Receding Horizon Control Loop
    for k = 1:N_steps
        t_now = t_steps(k);
        
        % Horizon Reference Sampling
        % Discretizes the reference trajectory over the prediction horizon p.
        xref_seq = zeros(10, cp.p + 1);
        uref_seq = zeros(4, cp.p);
        for j = 0:cp.p
            [z_h, dz_h, ddz_h] = traj_obj.get_flat_outputs(t_now + j*cp.Ts);
            [xr_h, ur_h] = FlatnessMap.map(z_h, dz_h, ddz_h);
            xref_seq(:, j+1) = xr_h;
            if j < cp.p, uref_seq(:, j+1) = ur_h; end
        end
        
        % Terminal Cost Calculation
        % For non-static trajectories, Qf is re-calculated at each 
        % step using the terminal reference point to provide a valid cost-to-go.
        if cp.use_computed_terminal_cost && ~strcmp(scenario_name, 'Hover')
            x_term = xref_seq(:, end);
            u_term = uref_seq(:, end);
            mpc.Qf = TerminalCost.compute(Q_mat, R_mat, mpc.model, cp.Ts, x_term, u_term);
        end
        
        % Solve LTV-MPC Problem (Sparse QP)
        tic;
        [u_opt, debug_info] = mpc.solve(current_x, xref_seq, uref_seq);
        calc_time = toc;
        sum_calc_time = sum_calc_time + calc_time;
        
        % Utilizing ODE45 for sub-sampling accuracy between control steps.
        tspan = [t_now, t_now + cp.Ts];
        [~, x_sol] = ode45(@(t,x) real_helicopter.dynamics(t, x, u_opt), tspan, current_x);
        next_x = x_sol(end, :)';
        
        % Logging (20 Channels)
        ref_now = xref_seq(:,1);
        res_data(k, :) = [
            t_now, ...                         % 1: Time
            current_x(1:4)', ...               % 2-5: Pos (X,Y,Z) & Yaw
            current_x(5:7)', ...               % 6-8: Body Velocities
            ref_now(1:4)', ...                 % 9-12: Reference States
            u_opt', ...                        % 13-16: Applied Inputs
            current_x(9:10)', ...              % 17-18: Integral Error States
            debug_info.cost, ...               % 19: Optimization Cost
            calc_time                          % 20: Solve Duration
        ];
        
        % Accumulate tracking performance (RMSE)
        sum_sq_err_x = sum_sq_err_x + norm(current_x(1:4) - ref_now(1:4))^2;
        current_x = next_x;
    end
    fprintf('Done.\n');
    
    % CSV Serialization 
    var_names = {
        'Time', 'X', 'Y', 'Z', 'Psi', 'Vx_B', 'Vy_B', 'Vz_B', ...
        'Ref_X', 'Ref_Y', 'Ref_Z', 'Ref_Psi', 'Ux', 'Uy', 'Uz', 'Upsi', ...
        'Int_X', 'Int_Y', 'Cost', 'CalcTime'
    };
    T_res = array2table(res_data, 'VariableNames', var_names);
    csv_name = sprintf('MPC_Results_%s.csv', scenario_name);
    writetable(T_res, fullfile(fullRunDir, csv_name));
    
    % Compute metrics
    mse_x = sum_sq_err_x / N_steps;
    rmse_x = sqrt(mse_x);
    metrics_data(end+1, :) = {scenario_name, mse_x, rmse_x, sum_calc_time/N_steps, N_steps, T_end};
end

%% FINAL PERFORMANCE SUMMARY EXPORT

fprintf('      SIMULATION COMPLETE: SUMMARY METRICS         \n');

var_met = {'Scenario', 'State_MSE', 'State_RMSE', 'Avg_SolveTime', 'Steps', 'Duration'};
T_met = cell2table(metrics_data, 'VariableNames', var_met);
writetable(T_met, fullfile(fullRunDir, 'MPC_Metrics_Summary.csv'));

disp(T_met); 
fprintf('[SUCCESS] Global summary archived in: %s\n', fullRunDir);