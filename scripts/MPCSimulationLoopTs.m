% MPC Simulation Loop for Ts analysis
%
% Compare Circle vs Spiral with two different strategies for starting points:
%
% Origin = helicopter placed at the origin.
% Deviation = Offset of 15 cm on x-axis (from initial point of reference trajectory)
%
% Analyze different Ts [0.02, 0.05, 0.1, 0.2].

clear; clc; close all;

%% SETUP ENVIRONMENT
[scriptDir, ~, ~] = fileparts(mfilename('fullpath'));
projectRoot = fileparts(scriptDir);
resultsDir = fullfile(projectRoot, 'results/MPC/Ts-Analysis');
if ~exist(resultsDir, 'dir'), mkdir(resultsDir); end

% Simulation Settings
Ts_values = [0.02, 0.05, 0.1, 0.2];
model_params = ModelParams;
traj_params = TrajectoryParams;

% Initialize Metrics Data
metrics_data = {}; 
colors = lines(length(Ts_values)); 

%% SCREEN SIZE DETECTION & LAYOUT LOGIC
screenSize = get(0, 'ScreenSize');
scrW = screenSize(3);
scrH = screenSize(4);

% width, height of figures
figW = scrW * 0.90;
figH = scrH * 0.85;

% plot fixed -> bug correction for final plots
centerPos = @(offset_x, offset_y) [ ...
    (scrW - figW)/2 + offset_x, ...      % Left
    max(50, (scrH - figH)/2 - offset_y), ... % Bottom (
    figW, ...                            % Width
    figH ];                              % Height

%% DEFINITIONS
scenarios = {
    'Circle', ShapeCircle(traj_params.Circle), traj_params.T_duration_circle, '2D';
    'Spiral', ShapeSpiral(traj_params.Spiral), traj_params.T_duration_spiral, '3D'
};

init_cond_cases = {
    'Origin',    1;
    'Deviation', 2  
};

% Error Monitoring 
fig_monitoring = figure('Name', 'C - Error Monitoring', ...
    'Position', centerPos(0, 0));

% Linearization Error Analysis Figure
fig_lin_error = figure('Name', 'Linearization Error Analysis', ...
    'Position', centerPos(80, 80));

%% MAIN SIMULATION LOOPS
for s = 1:size(scenarios, 1)
    
    scen_name = scenarios{s, 1};
    traj_obj  = scenarios{s, 2};
    T_end     = scenarios{s, 3};
    plot_type = scenarios{s, 4};
    
    fprintf('\n SCENARIO: %s \n', scen_name);
    
    % figure positioning
    if strcmp(scen_name, 'Circle')
        fig_circle = figure('Name', 'Circle Trajectories', ...
            'Position', centerPos(20, 20));
    else
        fig_spiral_orig = figure('Name', 'Spiral - Origin', ...
            'Position', centerPos(40, 40));
            
        fig_spiral_dev  = figure('Name', 'Spiral - Deviation', ...
            'Position', centerPos(60, 60));
    end
    
    % Loop over Initial Conditions
    for c = 1:size(init_cond_cases, 1)
        ic_name = init_cond_cases{c, 1};
        sp_idx  = init_cond_cases{c, 2}; 
        
        fprintf('Condition: %s\n', ic_name);
        
        ax_circle = [];
        if strcmp(scen_name, 'Circle')
            figure(fig_circle);
            ax_circle = subplot(1, 2, sp_idx);
            hold(ax_circle, 'on'); grid(ax_circle, 'on');
            title(ax_circle, sprintf('Circle - %s', ic_name));
            xlabel(ax_circle, 'X [m]'); ylabel(ax_circle, 'Y [m]');
            axis(ax_circle, 'equal');
            draw_reference(traj_obj, T_end, ax_circle, plot_type);
        end
        
        % Origin: Initial condition at [0; 0; 0; 0; 0; 0; 0; 0; 0; 0]; 
        if strcmp(ic_name, 'Origin')
            x0 = zeros(10, 1);
            xref0 = x0;
        else
            [z0, dz0, ddz0] = traj_obj.get_flat_outputs(0);
            [xref0, ~] = FlatnessMap.map(z0, dz0, ddz0); % get xref with differential flatness
            offset = [0.15; 0.0; 0.0; 0; 0; 0; 0; 0; 0; 0]; % offset -> deviation from initial point on the reference 15cm on x-axis for both shapes
            x0 = xref0 + offset; % apply offsett to initial ref. point

        end
        
        % Monitoring Axes 
        mon_idx = (s-1)*2 + sp_idx;
        figure(fig_monitoring);
        ax_err = subplot(2, 2, mon_idx);
        hold(ax_err, 'on'); grid(ax_err, 'on');
        title(ax_err, sprintf('%s - %s', scen_name, ic_name));
        xlabel(ax_err, 'Time [s]'); ylabel(ax_err, 'Pos Error [m]');
        ylim(ax_err, [0, 2.0]);
        
        % Linearization Error 
        figure(fig_lin_error);
        ax_lin = subplot(2, 2, mon_idx);
        hold(ax_lin, 'on'); grid(ax_lin, 'on');
        title(ax_lin, sprintf('Lin. Error: %s - %s', scen_name, ic_name));
        xlabel(ax_lin, 'Time [s]'); 
        ylabel(ax_lin, '|| \Delta x_{NL} - \Delta x_{Lin} || ');
        
        % LOOP OVER SAMPLING TIMES = [0.02, 0.05, 0.1, 0.2].
        for idx_ts = 1:length(Ts_values)
            current_Ts = Ts_values(idx_ts);
            
            % Temporary dictionary copying ControlParams
            base_cp = ControlParams;
            cp_mock = struct();
            cp_mock.Ts = current_Ts;
            cp_mock.p  = base_cp.p; 
            cp_mock.Q_diag = base_cp.Q_diag;
            cp_mock.R_diag = base_cp.R_diag;
            cp_mock.use_computed_terminal_cost = base_cp.use_computed_terminal_cost;
            
            % Init System
            mpc = MPCControllerv2(HelicopterModel(false));
            mpc.cp = cp_mock; 
            mpc.Q = diag(cp_mock.Q_diag);
            mpc.R = diag(cp_mock.R_diag);
            mpc.Qf = mpc.Q; 
            
            real_model = HelicopterModel(false);
            
            % Init MPC on starting point (origin / deviation from refrence)
            mpc.init(xref0, traj_obj, 0);
            
            % Simulation Loop
            t_steps = 0:current_Ts:T_end;
            N_steps = length(t_steps);
            
            pos_history = zeros(3, N_steps);
            err_history = zeros(1, N_steps);
            lin_err_history = zeros(1, N_steps); % Store linearization error
            
            sq_err_sum = 0;
            x_current = x0;
            
            for k = 1:N_steps
                t_now = t_steps(k);
                
                % Reference Horizon
                xref_seq = zeros(10, cp_mock.p + 1);
                uref_seq = zeros(4, cp_mock.p);
                for j = 0:cp_mock.p
                    [z, dz, ddz] = traj_obj.get_flat_outputs(t_now + j*current_Ts);
                    [xr, ur] = FlatnessMap.map(z, dz, ddz);
                    xref_seq(:, j+1) = xr;
                    if j < cp_mock.p, uref_seq(:, j+1) = ur; end
                end
                
                % Terminal Cost
                if cp_mock.use_computed_terminal_cost
                   mpc.Qf = TerminalCost.compute(mpc.Q, mpc.R, mpc.model, current_Ts, xref_seq(:,end), uref_seq(:,end));
                end
                
                % Solve
                try
                    [u_opt, ~] = mpc.solve(x_current, xref_seq, uref_seq);
                catch
                    u_opt = zeros(4,1); 
                end
                
                % Linearization Error Calculation 
                ref_now = xref_seq(:, 1); 
                
                % Non-linear update (True Dynamics)
                dxdt_nl = real_model.dynamics(0, x_current, u_opt, ref_now);
                
                % Linear update (LTI Approximation at current point)
                % x_dot ~ A*x + B*u
                [Ac, Bc] = real_model.get_jacobians(x_current, u_opt);
                dxdt_lin = Ac * x_current + Bc * u_opt;
                
                % Error Measure: Discrepancy accumulated over Ts
                % This shows how much the non-linear reality diverges from 
                % the linear assumption used in matrices, scaled by the hold time.
                lin_err_history(k) = norm(dxdt_nl - dxdt_lin) * current_Ts;

                % Nonlinear Simulation (Explicit Euler)
                % Note: PW Constant inputs are used between each sampling
                % [(kTs), (k+1)Ts ]
                x_next = x_current + dxdt_nl * current_Ts;
                
                % Log
                pos_history(:, k) = x_current(1:3);
                err_val = norm(x_current(1:3) - ref_now(1:3));
                err_history(k) = err_val;
                sq_err_sum = sq_err_sum + err_val^2;
                x_current = x_next;
            end
            
            % Metrics: MSE, RMSE
            mse = sq_err_sum / N_steps;
            rmse = sqrt(mse);
            metrics_data(end+1, :) = {scen_name, ic_name, current_Ts, mse, rmse};
            
            if strcmp(scen_name, 'Circle')
                % Circle: Plot on the shared axes
                plot(ax_circle, pos_history(1,:), pos_history(2,:), ...
                    'LineWidth', 1.2, 'Color', colors(idx_ts,:), ...
                    'DisplayName', sprintf('Ts=%.2f', current_Ts));
            else
                % Spiral: Select correct Figure and Subplot
                if strcmp(ic_name, 'Origin')
                    figure(fig_spiral_orig);
                else
                    figure(fig_spiral_dev);
                end

                if idx_ts == 1
                    tiledlayout(2,2,'TileSpacing','compact','Padding','compact');
                end

                ax_spiral = nexttile;
                hold(ax_spiral, 'on'); grid(ax_spiral, 'on');
                title(ax_spiral, sprintf('Ts = %.2f s', current_Ts), 'FontSize', 13);
                xlabel(ax_spiral, 'X [m]');
                ylabel(ax_spiral, 'Y [m]');
                zlabel(ax_spiral, 'Z [m]');
                view(ax_spiral, 45, 30);
                axis(ax_spiral, 'vis3d');

                draw_reference(traj_obj, T_end, ax_spiral, plot_type);
                plot3(ax_spiral, pos_history(1,:), pos_history(2,:), pos_history(3,:), ...
                    'LineWidth', 1.6, 'Color', colors(idx_ts,:));
            end
            
            % Plot Error 
            plot(ax_err, t_steps, err_history, ...
                'LineWidth', 1.2, 'Color', colors(idx_ts,:), ...
                'DisplayName', sprintf('Ts=%.2f', current_Ts));
            
            % Plot Linearization Error
            plot(ax_lin, t_steps, lin_err_history, ...
                'LineWidth', 1.2, 'Color', colors(idx_ts,:), ...
                'DisplayName', sprintf('Ts=%.2f', current_Ts));
            
        end % End Ts
        
        % Legends for Circle
        if strcmp(scen_name, 'Circle')
            legend(ax_circle, 'show', 'Location', 'bestoutside');
        end
        % Legends for Error
        legend(ax_err, 'show', 'Location', 'bestoutside');
        legend(ax_lin, 'show', 'Location', 'bestoutside');
        
    end 
    
    % Save Figures
    %if strcmp(scen_name, 'Circle')
        %savefig(fig_circle, fullfile(resultsDir, 'Plot_Circle_All.fig'));
    %else
        %savefig(fig_spiral_orig, fullfile(resultsDir, 'Plot_Spiral_Origin_Split.fig'));
        %savefig(fig_spiral_dev, fullfile(resultsDir, 'Plot_Spiral_Deviation_Split.fig'));
    %end
    
end % End Scenario

% Save Error Monitoring Figure
%savefig(fig_monitoring, fullfile(resultsDir, 'Plot_Error_Monitoring.fig'));
%savefig(fig_lin_error, fullfile(resultsDir, 'Plot_Linearization_Error.fig'));

%% EXPORT CSV
fprintf('\n EXPORTING METRICS: \n');
varNames = {'Scenario', 'Start_Condition', 'Ts', 'MSE_Position', 'RMSE_Position'};
T_results = cell2table(metrics_data, 'VariableNames', varNames);
disp(T_results);

csv_filename = fullfile(resultsDir, 'MPC_Simulation_Results.csv');
writetable(T_results, csv_filename);
fprintf('Metrics saved to: %s\n', csv_filename);


%% HELPER FUNCTIONS
% draw reference given:
% traj_obj: shape
% T_end: simulation duration
% ax: selected axis
% type: 3D, 2D

function draw_reference(traj_obj, T_end, ax, type)
    t_fine = 0:0.01:T_end;
    ref_points = zeros(3, length(t_fine));
    for i = 1:length(t_fine)
        [z_f, ~, ~] = traj_obj.get_flat_outputs(t_fine(i));
        ref_points(:, i) = z_f(1:3); 
    end
    
    if strcmp(type, '3D')
        plot3(ax, ref_points(1,:), ref_points(2,:), ref_points(3,:), ...
            'k--', 'LineWidth', 1.0, 'DisplayName', 'Reference');
    else
        plot(ax, ref_points(1,:), ref_points(2,:), ...
            'k--', 'LineWidth', 1.0, 'DisplayName', 'Reference');
    end
end