% SIMULATION ORCHESTRATOR: LTV-MPC FOR MINIATURE HELICOPTERS
% Evaluates the performance of a Receding Horizon controller across 

clear; clc; close all;

%% ENVIRONMENTAL SETUP & DIRECTORY VERSIONING
[scriptDir, ~, ~] = fileparts(mfilename('fullpath'));
projectRoot = fileparts(scriptDir);

resultsBaseDir = fullfile(projectRoot, 'results/MPC/PWC_reference/');
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

fprintf('      MINIATURE HELICOPTER MPC SIMULATION WITH PWC REFERENCE          \n');
fprintf('[SYSTEM] Output Directory: %s\n', runDirName);

%% SYSTEM INITIALIZATION
mp = ModelParams;
cp = ControlParams;
tp = TrajectoryParams;

Ts = cp.Ts; 
options = odeset('RelTol', 1e-8, 'AbsTol', 1e-9);

% Model and MPC initialization
real_helicopter = HelicopterModel(false);
mpc = MPCController_PWC_reference(HelicopterModel(false));

%% TRAJECTORY DEFINITIONS
% Circle
circle_config = tp.Circle;
circle_config.yaw_mode = 'tangent';
traj_circle = ShapeCircle(circle_config);

% Spiral
spiral_config = tp.Spiral;
spiral_config.yaw_mode = 'tangent';
traj_spiral = ShapeSpiral(spiral_config);

% Ellipse Spiral
ellipse_config = tp.Ellipse;
ellipse_config.yaw_mode = 'tangent';
traj_ellipse = ShapeSpiralEllipse(ellipse_config);

% Lemniscate
lemniscate_config = tp.Lemniscate;
traj_lemniscate = ShapeLemniscate(lemniscate_config);

% Generate PWC references
[x_circle, u_circle, ~]         = GeneratePWC_reference(traj_circle, tp.T_duration_circle + 10, Ts, HelicopterModel, options);
[x_lemniscate, u_lemniscate, ~] = GeneratePWC_reference(traj_lemniscate, tp.T_duration_lemniscate + 10, Ts, HelicopterModel, options);
[x_spiral, u_spiral, ~]         = GeneratePWC_reference(traj_spiral, tp.T_duration_spiral + 10, Ts, HelicopterModel, options);
[x_ellipse, u_ellipse, ~]       = GeneratePWC_reference(traj_ellipse, tp.T_duration_ellipse + 10, Ts, HelicopterModel, options);

%% SCENARIOS
scenarios = {
    'Circle',          traj_circle,     tp.T_duration_circle,     x_circle,     u_circle,     '2D';
    'Lemniscate',      traj_lemniscate, tp.T_duration_lemniscate, x_lemniscate, u_lemniscate, '2D';
    'Spiral',          traj_spiral,     tp.T_duration_spiral,     x_spiral,     u_spiral,     '3D';
    'SpiralEllipse',  traj_ellipse,    tp.T_duration_ellipse,    x_ellipse,    u_ellipse,    '3D'
};

metrics_data = {};

%% EXPORT JSON CONFIGURATION
configStruct.ModelParams      = struct('bx', mp.bx, 'by', mp.by, 'bz', mp.bz, 'kx', mp.kx, 'ky', mp.ky, 'u_min', mp.u_min, 'u_max', mp.u_max);
configStruct.ControlParams    = struct('Ts', cp.Ts, 'Horizon_p', cp.p, 'Q_diag', cp.Q_diag, 'R_diag', cp.R_diag, 'Stability_Enhanced_Qf', cp.use_computed_terminal_cost);
configStruct.TrajectoryParams = struct('Hover', tp.Hover, 'Circle', tp.Circle, 'Lemniscate', tp.Lemniscate, 'Spiral', tp.Spiral, 'SpiralEllipse', tp.Ellipse);

jsonStr = jsonencode(configStruct, 'PrettyPrint', true);
fid = fopen(fullfile(fullRunDir, 'simulation_config.json'), 'w');
if fid ~= -1
    fwrite(fid, jsonStr, 'char');
    fclose(fid);
end
fprintf('[SYSTEM] Global system configuration exported to JSON.\n');

%% SIMULATION PIPELINE
initial_conditions = {'OnReference', 'Perturbation', 'Origin'};

% Initial Conditions
for c = 1:length(initial_conditions)
    cond_name = initial_conditions{c};
    
    % Flight Scenarios
    for s = 1:size(scenarios, 1)
        
        scenario_name = scenarios{s, 1};
        traj_obj      = scenarios{s, 2};
        T_end         = scenarios{s, 3}; 
        x_ref_pwc     = scenarios{s, 4};
        u_ref_pwc     = scenarios{s, 5};
        plot_mode     = scenarios{s, 6}; 

        fprintf('\n[SCENARIO] %s | Condition: %s (Duration: %.1fs)\n', scenario_name, cond_name, T_end);

        % Compute nominal starting point
        [z0, dz0, ddz0] = traj_obj.get_flat_outputs(0);
        [x0_nominal, ~] = FlatnessMap.map(z0, dz0, ddz0);
        current_x = x0_nominal(:);

        % Apply initial conditions and init MPC
        if strcmp(cond_name, 'OnReference')
            mpc.init(current_x, x_ref_pwc, u_ref_pwc);

        elseif strcmp(cond_name, 'Perturbation') 
            current_x(1) = current_x(1) + 0.15; 
            mpc.init(current_x, x_ref_pwc, u_ref_pwc);

        elseif strcmp(cond_name, 'Origin') && ~strcmp(scenario_name, 'Lemniscate')
            current_x = zeros(10, 1);
            x_ref_zeros = zeros(size(x_ref_pwc));
            u_ref_zeros = zeros(size(u_ref_pwc));
            mpc.init(current_x, x_ref_zeros, u_ref_zeros);

        elseif strcmp(cond_name, 'Origin') && strcmp(scenario_name, 'Lemniscate')
            current_x = zeros(10, 1);
            current_x(1) = 0.75; 
            x_ref_start = zeros(size(x_ref_pwc));
            x_ref_start(1, :) = 0.75; 
            u_ref_zeros = zeros(size(u_ref_pwc));
            mpc.init(current_x, x_ref_start, u_ref_zeros);
        end
        
        % Temporal discretization
        t_steps = 0:cp.Ts:T_end;
        N_steps = length(t_steps);
        res_data = zeros(N_steps, 20);

        sum_sq_err_x = 0;   
        sum_sq_err_u = 0;
        sum_calc_time = 0;  

        fprintf('    Simulating... ');

        % Hover state extraction for terminal horizon
        x_hover = x_ref_pwc(:, end);
        x_hover(5:10) = 0; % Null velocities and integral states
        u_hover = [0.0; 0.0; mp.g/mp.bz; 0.0]; % Gravity compensation

        % Receding Horizon Loop
        for k = 1:N_steps
            t_now = t_steps(k);
            
            xref_seq = zeros(10, cp.p + 1);
            uref_seq = zeros(4, cp.p);

            % Populate horizon reference
            for j = 0:cp.p
                idx = k + j;
                
                if idx <= size(x_ref_pwc, 2)
                    xref_seq(:, j+1) = x_ref_pwc(:, idx); 
                    if j < cp.p
                        uref_seq(:, j+1) = u_ref_pwc(:, idx);
                    end
                else
                    % Beyond reference: apply hover state
                    xref_seq(:, j+1) = x_hover;
                    if j < cp.p
                        uref_seq(:, j+1) = u_hover;
                    end
                end
            end

            % Solve LTV-MPC Sparse QP
            tic;
            [u_opt, debug_info] = mpc.solve(current_x, xref_seq, uref_seq);
            calc_time = toc;
            sum_calc_time = sum_calc_time + calc_time;

            % Apply optimal input to nonlinear plant (ODE45)
            tspan = [t_now, t_now + cp.Ts];
            [~, x_sol] = ode45(@(t,x) real_helicopter.dynamics(t, x, u_opt), tspan, current_x);
            next_x = x_sol(end, :)';

            % Logging
            ref_now = xref_seq(:, 1);
            res_data(k, :) = [
                t_now, current_x(1:4)', current_x(5:7)', ref_now(1:4)', ...
                u_opt', current_x(9:10)', debug_info.cost, calc_time
            ];

            % Performance accumulation
            sum_sq_err_x = sum_sq_err_x + norm(current_x(1:4) - ref_now(1:4))^2;
            sum_sq_err_u = sum_sq_err_u + norm(u_opt - uref_seq(:, 1))^2;

            current_x = next_x;
            
        end
        fprintf('Done.\n');

        % Data Serialization
        var_names = {'Time', 'X', 'Y', 'Z', 'Psi', 'Vx_B', 'Vy_B', 'Vz_B', ...
                     'Ref_X', 'Ref_Y', 'Ref_Z', 'Ref_Psi', 'Ux', 'Uy', 'Uz', 'Upsi', ...
                     'Int_X', 'Int_Y', 'Cost', 'CalcTime'};
        
        T_res = array2table(res_data, 'VariableNames', var_names);
        csv_name = sprintf('MPC_Results_%s_%s.csv', scenario_name, cond_name);
        writetable(T_res, fullfile(fullRunDir, csv_name));

        % GIF Generation
        gif_name = fullfile(fullRunDir, sprintf('Tracking_%s_%s.gif', scenario_name, cond_name));
        title_str = sprintf('%s (Ts = %.2fs) - %s', scenario_name, cp.Ts, cond_name);
        generateSimulationGIF(res_data, x_ref_pwc, gif_name, title_str, plot_mode);
        
        % Aggregate Metrics
        mse_x = sum_sq_err_x / N_steps;
        rmse_x = sqrt(mse_x);
        mse_u = sum_sq_err_u / N_steps;
        metrics_data(end+1, :) = {cond_name, scenario_name, mse_x, rmse_x, mse_u, sum_calc_time/N_steps, N_steps, T_end};

    end
end

%% FINAL PERFORMANCE SUMMARY EXPORT
fprintf('\n      SIMULATION COMPLETE: SUMMARY METRICS         \n');

var_met = {'Condition', 'Scenario', 'State_MSE', 'State_RMSE', 'Input_MSE', 'Avg_SolveTime', 'Steps', 'Duration'};
T_met = cell2table(metrics_data, 'VariableNames', var_met);
writetable(T_met, fullfile(fullRunDir, 'MPC_Metrics_Summary.csv'));

disp(T_met); 
fprintf('[SUCCESS] Global summary archived in: %s\n', fullRunDir);


%% HELPER FUNCTIONS
function generateSimulationGIF(res_data, x_ref_pwc, gif_filename, scenario_name, mode)
    % Extracts data columns
    X_sim = res_data(:, 2);
    Y_sim = res_data(:, 3);
    Z_sim = res_data(:, 4);

    N = length(X_sim);
    
    % Limits reference rendering to the common simulated horizon
    idx_end = min(N, size(x_ref_pwc, 2));
    X_ref = x_ref_pwc(1, 1:idx_end);
    Y_ref = x_ref_pwc(2, 1:idx_end);
    Z_ref = x_ref_pwc(3, 1:idx_end);

    % Figure Setup
    figure('Color','w', 'Position', [100 100 1400 900]);
    hold on; grid on; axis equal;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    title(['3D Tracking - ' scenario_name], 'FontSize', 16);

    if strcmp(mode, '2D')
        view(2); 
    else
        view(3); 
    end

    % Graphic Object Handles
    plot3(X_ref, Y_ref, Z_ref, 'k--', 'LineWidth', 2);
    h_traj = plot3(NaN, NaN, NaN, 'Color', [0 0.4 1], 'LineWidth', 2.5);
    h_heli = plot3(NaN, NaN, NaN, 'o', 'MarkerSize', 10, 'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'k');
    h_target = plot3(NaN, NaN, NaN, 'x', 'MarkerSize', 14, 'LineWidth', 2.5, 'Color', 'k');

    axis padded;
    set(gca, 'FontSize', 14);

    delay = 0.05;

    % Frame Rendering Loop
    for k = 1:N
        set(h_traj, 'XData', X_sim(1:k), 'YData', Y_sim(1:k), 'ZData', Z_sim(1:k));
        set(h_heli, 'XData', X_sim(k), 'YData', Y_sim(k), 'ZData', Z_sim(k));

        idx = min(k, idx_end);
        set(h_target, 'XData', X_ref(idx), 'YData', Y_ref(idx), 'ZData', Z_ref(idx));

        drawnow;

        % Capture and append frame
        frame = getframe(gcf);
        im = frame2im(frame);
        [A, map] = rgb2ind(im, 256);

        if k == 1
            imwrite(A, map, gif_filename, 'gif', 'LoopCount', inf, 'DelayTime', delay);
        else
            imwrite(A, map, gif_filename, 'gif', 'WriteMode', 'append', 'DelayTime', delay);
        end
    end
    close(gcf);
end