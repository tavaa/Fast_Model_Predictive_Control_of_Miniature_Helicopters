% PID SIMULATION ORCHESTRATOR
%
% This script serves as the primary benchmark execution environment for 
% the Proportional-Integral-Derivative (PID) controller. It evaluates 
% tracking performance against a high-fidelity nonlinear plant model.
%
% Pipeline:
%   1. Environmental Setup: Dynamic dependency mapping and versioned logging.
%   2. Configuration: Initialization of physical constants and PID gains.
%   3. Closed-Loop Execution: Point-to-point tracking via ODE45 integration.
%   4. Metric Extraction: Calculation of MSE, RMSE, and temporal efficiency.
%   5. Data Serialization: Export of CSV and JSON.

clear; clc; close all;

%% ENVIRONMENT & DIRECTORY SETUP
[scriptDir, ~, ~] = fileparts(mfilename('fullpath'));
projectRoot = fileparts(scriptDir);

% Recursive Results Directory Management 
resultsBaseDir = fullfile(projectRoot, 'results', 'PID');
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

fprintf('      HELICOPTER PID BENCHMARK LOOP           \n');
fprintf('[INFO] Initializing run directory: %s\n', runDirName);

%% SYSTEM CONFIGURATION
% Instance generation for parameters and models
mp = ModelParams;
cp = ControlParams;
tp = TrajectoryParams;

% Plant instance utilizing full nonlinear physics (no Jacobian simplification)
real_helicopter = HelicopterModel(false); 

% Controller Instantiation 
% PID controller utilizing gains from ControlParams
pid = PIDController();

% Evaluation Scenarios: {Identifier, Object, Duration}
scenarios = {
    'Hover',  ShapeHover(tp.Hover),   tp.T_duration_hover;
    'Circle', ShapeCircle(tp.Circle), tp.T_duration_circle;
    'Spiral', ShapeSpiral(tp.Spiral), tp.T_duration_spiral
};

metrics_data = {};

%% EXPORT CONFIGURATION (JSON)
% Serialization of current parameters for reproducibility.
configStruct.ModelParams = struct(...
    'bx', mp.bx, 'by', mp.by, 'bz', mp.bz, ...
    'u_min', mp.u_min, 'u_max', mp.u_max);

% Captures gains defined in the ControlParams class
configStruct.PID_Gains = struct(...
    'Kp', cp.PID_Kp, 'Ki', cp.PID_Ki, 'Kd', cp.PID_Kd, ...
    'Note', 'Reference tuning used for benchmark comparison');

configStruct.TrajectoryParams = struct(...
    'Hover', tp.Hover, 'Circle', tp.Circle, 'Spiral', tp.Spiral);

jsonStr = jsonencode(configStruct, 'PrettyPrint', true);
fid = fopen(fullfile(fullRunDir, 'pid_config.json'), 'w');
if fid == -1, error('Failed to initialize JSON metadata file.'); end
fwrite(fid, jsonStr, 'char');
fclose(fid);

%% MAIN SIMULATION LOOP
for s = 1:size(scenarios, 1)
    
    scenario_name = scenarios{s, 1};
    traj_obj      = scenarios{s, 2};
    T_end         = scenarios{s, 3};
    
    fprintf('\n>>> Running Scenario: %s (Duration: %.1fs)...\n', scenario_name, T_end);
    
    % Initial State Synchronization (Cold Start) 
    [z0, dz0, ddz0] = traj_obj.get_flat_outputs(0);
    [x0, ~] = FlatnessMap.map(z0, dz0, ddz0);
    
    current_x = x0;
    pid.reset(); % Initialize integral memory to zero
    
    % Temporal discretization
    t_steps = 0:cp.Ts:T_end;
    N_steps = length(t_steps);
    
    % Preallocation (21 columns)
    data_log = zeros(N_steps, 21);
    
    % Metric accumulators
    sum_sq_err_x = 0; 
    sum_sq_err_u = 0; 
    sum_calc_time = 0;
    
    fprintf('    Simulating... ');
    
    % Closed-Loop Integration Loop
    for k = 1:N_steps
        t_now = t_steps(k);
        
        % Reference Generation (Analytic trajectory sampling)
        [z, dz, ddz] = traj_obj.get_flat_outputs(t_now);
        [x_ref_now, u_ref_now] = FlatnessMap.map(z, dz, ddz);
        
        % Control Law Evaluation
        tic;
        [u_opt, debug_info] = pid.compute(current_x, x_ref_now);
        calc_time = toc;
        sum_calc_time = sum_calc_time + calc_time;
        
        % Numerical Plant Propagation
        % Utilizing ODE45 for inter-step sub-sampling accuracy.
        tspan = [t_now, t_now + cp.Ts];
        [~, x_sol] = ode45(@(t,x) real_helicopter.dynamics(t, x, u_opt), tspan, current_x);
        next_x = x_sol(end, :)';
        
        % Error Quantification (Euclidean norm of tracking error)
        err_state_vec = current_x(1:4) - x_ref_now(1:4);
        sum_sq_err_x = sum_sq_err_x + norm(err_state_vec)^2;
        
        err_input_vec = u_opt - u_ref_now;
        sum_sq_err_u = sum_sq_err_u + norm(err_input_vec)^2;
        
        % Log [Time, State, Velocity, Reference, Input, Integrals, SolveTime]
        data_log(k, :) = [
            t_now, ...                                                  
            current_x(1), current_x(2), current_x(3), current_x(4), ... 
            current_x(5), current_x(6), current_x(7), ...               
            x_ref_now(1), x_ref_now(2), x_ref_now(3), x_ref_now(4), ... 
            u_opt(1),     u_opt(2),     u_opt(3),     u_opt(4),     ... 
            debug_info.int(1), debug_info.int(2), ...                   
            debug_info.int(3), debug_info.int(4), ...                   
            calc_time                                                   
        ];
        
        current_x = next_x;
    end
    fprintf('Done.\n');
    
    % Scenario Result Serialization
    var_names = {
        'Time', 'X', 'Y', 'Z', 'Psi', 'Vx_B', 'Vy_B', 'Vz_B', ...
        'Ref_X', 'Ref_Y', 'Ref_Z', 'Ref_Psi', 'Ux', 'Uy', 'Uz', 'Upsi', ...
        'Err_Int_X', 'Err_Int_Y', 'Err_Int_Z', 'Err_Int_Psi', 'CalcTime'
    };
    
    T_res = array2table(data_log, 'VariableNames', var_names);
    csv_name = sprintf('PID_Results_%s.csv', scenario_name);
    writetable(T_res, fullfile(fullRunDir, csv_name));
    
    % Aggregate performance metrics calculation
    mse_x = sum_sq_err_x / N_steps;
    mse_u = sum_sq_err_u / N_steps;
    rmse_x = sqrt(mse_x);
    avg_calc_time = sum_calc_time / N_steps;
    
    metrics_data(end+1, :) = {scenario_name, mse_x, rmse_x, mse_u, avg_calc_time, N_steps, T_end};
end

%% EXPORT METRICS SUMMARY
fprintf('\n Serializing PID Performance Metrics...\n');

var_met = {'Shape', 'State_MSE', 'State_RMSE', 'Input_MSE', 'Avg_CalcTime', 'Steps', 'Duration'};
T_met = cell2table(metrics_data, 'VariableNames', var_met);

file_met = fullfile(fullRunDir, 'PID_Metrics_Summary.csv');
writetable(T_met, file_met);

fprintf('[SUCCESS] Simulation results archived in: %s\n', fullRunDir);
disp(T_met); 