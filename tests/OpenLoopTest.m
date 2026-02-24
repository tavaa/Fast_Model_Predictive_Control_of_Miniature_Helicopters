% OPEN LOOP: CONTINUOUS vs PWC INPUTS COMPARISON
%
% Continuous control Input u(t) applied to Nonlinear Model
% Piece-Wise Constant Input u_k (hold constant in [kTs, (k+1)*Ts])
%

clear; clc; close all;

%% CONFIGS
Ts_values = [0.02, 0.20, 0.50, 0.75];
colors = {'b', 'g', 'm', 'r'}; 

% Nonlinear model
model = HelicopterModel(false);

% Trajectory parameters
tp = TrajectoryParams;

% Circle 
circle_config = tp.Circle;
circle_config.yaw_mode = 'tangent'; 

% Spiral
spiral_config = tp.Spiral;
spiral_config.yaw_mode = 'tangent'; 

% Ellipse 
ellipse_config = tp.Ellipse;
ellipse_config.yaw_mode = 'tangent';

% Lemniscate 
lemniscate_config = tp.Lemniscate;

% Shapes
traj_circle  = ShapeCircle(circle_config);
traj_spiral  = ShapeSpiral(spiral_config);
traj_ellipse = ShapeSpiralEllipse(ellipse_config);
traj_lemniscate = ShapeLemniscate(lemniscate_config);

% Define Scenarios to process
scenarios = {
    'Circle',  traj_circle,  tp.T_duration_circle,  '2D';
    'Lemniscate', traj_lemniscate, tp.T_duration_lemniscate, '2D';
    'Spiral',  traj_spiral,  tp.T_duration_spiral,  '3D';
    'Spiral Ellipse', traj_ellipse, tp.T_duration_ellipse, '3D'
};

%% MAIN LOOP OVER SCENARIOS
for s = 1:size(scenarios, 1)
    
    shape_name = scenarios{s, 1};
    traj_obj   = scenarios{s, 2};
    T_sim      = scenarios{s, 3};
    plot_type  = scenarios{s, 4};
    
    fprintf('\n Processing: %s \n', shape_name);

    % Initial state -> first point of reference trajectory
    [z0, dz0, ddz0] = traj_obj.get_flat_outputs(0);
    [x0, ~] = FlatnessMap.map(z0, dz0, ddz0);

    %% Continuous Inputs Simulation 
    % Continuous dynamics 
    dyn_cont = @(t,x) continuous_dynamics(t, x, model, traj_obj);
    options = odeset('RelTol', 1e-8, 'AbsTol', 1e-9);

    % Run continuous simulation
    [t_cont, x_cont] = ode45(dyn_cont, [0, T_sim], x0, options);
    
    %% PWC Simulations Loop
    % Store results for each Ts
    pwc_results = cell(1, length(Ts_values));
    
    for i = 1:length(Ts_values)
        curr_Ts = Ts_values(i);
        fprintf('Simulation PWC inputs Ts = %.2f s...\n', curr_Ts);
        
        t_steps = 0:curr_Ts:T_sim;
        N_steps = length(t_steps);
        
        x_hist = zeros(10, N_steps);
        
        x_hist(:,1) = x0;
        x_curr = x0;

        % PWC inputs
        for k = 1:N_steps-1
            t_now = t_steps(k);
            
            % Sampling
            [z, dz, ddz] = traj_obj.get_flat_outputs(t_now);
            [~, u_k] = FlatnessMap.map(z, dz, ddz);
            
            % create tspan [k*Ts, (k+1)*Ts]
            tspan = [t_now, t_now + curr_Ts];
            
            % get next state using fixed PWC input
            % ode45 integrates the physics correctly, but the input u_k is held constant
            [~, x_sol] = ode45(@(t, x) model.dynamics(t, x, u_k, zeros(10,1)), tspan, x_curr, options);
            
            x_curr = x_sol(end, :)';
            x_hist(:, k+1) = x_curr;
        end
        
        % Save comparison data
        res.Ts = curr_Ts;
        res.t = t_steps;
        res.x = x_hist;
        pwc_results{i} = res;
    end
    
    %% PLOTTING 
    fig_traj = figure('Name', sprintf('%s: Trajectory Comparison', shape_name), ...
        'Color', 'w', 'Position', [100, 100, 1000, 800]);
    
    for i = 1:length(Ts_values)
        res = pwc_results{i};
        ax = subplot(2, 2, i);
        hold(ax, 'on'); grid(ax, 'on');
        
        % Plot Continuous Reference (Black Dashed)
        if strcmp(plot_type, '2D')
            plot(ax, x_cont(:,1), x_cont(:,2), 'k--', 'LineWidth', 1.0, 'DisplayName', 'Continuous');
            plot(ax, res.x(1,:), res.x(2,:), '-', 'Color', colors{i}, 'LineWidth', 2.0, 'DisplayName', 'PWC');
            xlabel(ax, 'X [m]'); ylabel(ax, 'Y [m]');
        else
            plot3(ax, x_cont(:,1), x_cont(:,2), x_cont(:,3), 'k--', 'LineWidth', 1.0, 'DisplayName', 'Continuous');
            plot3(ax, res.x(1,:), res.x(2,:), res.x(3,:), '-', 'Color', colors{i}, 'LineWidth', 2.0, 'DisplayName', 'PWC');
            xlabel(ax, 'X [m]'); ylabel(ax, 'Y [m]'); zlabel(ax, 'Z [m]');
            view(ax, 30, 30);
        end
        axis(ax, 'equal');
        title(ax, sprintf('Ts = %.2f s', res.Ts));
        legend(ax, 'Location', 'best');
    end
    sgtitle(sprintf('%s Trajectory: Divergence of PWC Inputs', shape_name));
    
end

%% Continuous dynamics function
function dxdt = continuous_dynamics(t, x, model, traj)
    % Get Flat outputs and derivatives
    [z, dz, ddz] = traj.get_flat_outputs(t);
    % Get x, u reference using state/input reconstruction 
    [xref, u_cont] = FlatnessMap.map(z, dz, ddz);
    % Dynamics of nonlinear model 
    dxdt = model.dynamics(t, x, u_cont, xref);
end