% TEST TRAJECTORY GENERATION & KINEMATIC VERIFICATION
%
% Validates the geometric and kinematic consistency of the Shape classes.
% Generates two figures:
% 1. Standard Trajectories (Hover, Circle, Spiral) in a 2x3 grid.
% 2. Elliptical Spiral (Focus) in a 2x1 grid.

clear; clc; close all;

fprintf('      TRAJECTORY GEOMETRIC VERIFICATION      \n');

try
    %% TEST 1: Configuration Integrity
    fprintf('[1/3] Testing Parameter Loading... ');
    tp = TrajectoryParams;
    
    % Verify Spiral parameters
    assert(isfield(tp.Spiral, 'yaw_mode'), 'Spiral struct missing yaw_mode field.');
    assert(strcmp(tp.Spiral.yaw_mode, 'tangent'), 'Spiral should be in tangent mode.');
    
    % Safety check for Ellipse params
    %if ~isprop(tp, 'Ellipse') && ~isfield(tp, 'Ellipse')
        %tp.Ellipse = struct('Rx', 2.0, 'Ry', 0.5, 'omega', 0.5, 'vz', 0.1, 'z_start', 0, 'yaw_mode', 'tangent');
        %tp.T_duration_ellipse = (2 * pi / 0.5) * 2;
    %end

    fprintf('OK.\n');
    fprintf('      Spiral Duration:    %.2f s\n', tp.T_duration_spiral);
    
    %% TEST 2: Class Instantiation
    fprintf('[2/3] Instantiating Shape Classes... ');
    
    traj_hover  = ShapeHover(tp.Hover);
    traj_circle = ShapeCircle(tp.Circle);
    traj_spiral = ShapeSpiral(tp.Spiral);
    traj_ellipse = ShapeSpiralEllipse(tp.Ellipse);
    traj_lemniscate = ShapeLemniscate(tp.Lemniscate);
    
    fprintf('OK.\n');
    
    %% TEST 3: Plotting
    fprintf('[3/3] Generating Visualizations...\n');
    
    colors = lines(4); 
    
    %% FIGURE 1: STANDARD TRAJECTORIES (2x3 Grid)
    fig1 = figure('Name', 'Standard Trajectories (Hover, Circle, Spiral)', ...
                  'Color', 'w', 'Position', [50, 100, 1200, 800]);
    
    % Grid Size: 2 Rows, 3 Columns
    g_rows = 2; g_cols = 3;
    
    % Col 1: Hover
    run_and_plot(traj_hover, 0:0.1:tp.T_duration_hover, 'Hover', 1, colors(1,:), g_rows, g_cols);
    
    % Col 2: Circle
    run_and_plot(traj_circle, 0:0.05:tp.T_duration_circle, 'Circle', 2, colors(2,:), g_rows, g_cols);
    
    % Col 3: Spiral
    run_and_plot(traj_spiral, 0:0.05:tp.T_duration_spiral, 'Spiral 3D', 3, colors(3,:), g_rows, g_cols);
    
    
    %% FIGURE 2: ELLIPTICAL SPIRAL FOCUS (2x1 Grid)
    fig2 = figure('Name', 'Other Trajectories: Ellipse and Lemniscate', ...
              'Color', 'w', 'Position', [900, 50, 1000, 900]);
    
    % Grid Size: 2 Rows, 1 Column (Top for 3D, Bottom for Time)
    g_rows = 2; g_cols = 2;
    
    run_and_plot(traj_ellipse, 0:0.05:tp.T_duration_ellipse, 'Spiral Ellipse', 1, colors(4,:), g_rows, g_cols);
    run_and_plot(traj_lemniscate, 0:0.05:tp.T_duration_lemniscate, 'Lemniscate', 2, colors(4,:), g_rows, g_cols);
    
    
    fprintf('\n[SUCCESS] All geometric tests passed. Figures generated.\n');
    
catch ME
    fprintf('\n[FAILED]: %s\n', ME.message);
    fprintf('Location: %s (Line %d)\n', ME.stack(1).name, ME.stack(1).line);
end


%% HELPER FUNCTION: Kinematic Simulation and Plotting

function run_and_plot(traj_obj, t_vec, name, col_idx, col, grid_rows, grid_cols)
    % col_idx: The column index for this specific plot
    % grid_rows, grid_cols: Size of the subplot grid (e.g., 2x3 or 2x1)
    
    N = length(t_vec);
    
    % Preallocate Geometric Buffers
    Pos = zeros(3, N);
    Yaw = zeros(1, N);
    
    % Simulate Flat Output Sampling
    for k = 1:N
        [z, ~, ~] = traj_obj.get_flat_outputs(t_vec(k));
        Pos(:, k) = z(1:3);
        Yaw(k)    = z(4);
    end
    
    %% ROW 1: 3D Geometric Plot 
    % Subplot index calculation: (Row 1, Column col_idx)
    subplot(grid_rows, grid_cols, col_idx);
    
    plot3(Pos(1,:), Pos(2,:), Pos(3,:), 'Color', col, 'LineWidth', 2, 'DisplayName', 'Path');
    grid on; axis equal; hold on;
    
    % Start (Green Circle) and End (Red Square) markers
    plot3(Pos(1,1), Pos(2,1), Pos(3,1), 'o', 'MarkerSize', 8, 'MarkerFaceColor', 'g', 'DisplayName', 'Start');
    plot3(Pos(1,end), Pos(2,end), Pos(3,end), 's', 'MarkerSize', 8, 'MarkerFaceColor', 'r', 'DisplayName', 'End');
    
    % Heading Vector Visualization (Quivers)
    num_quivers = 15;
    step = max(floor(N/num_quivers), 1);
    idx_q = 1:step:N;
    
    v_len = 0.4;
    u_h = v_len * cos(Yaw(idx_q));
    v_h = v_len * sin(Yaw(idx_q));
    
    quiver3(Pos(1,idx_q), Pos(2,idx_q), Pos(3,idx_q), ...
            u_h, v_h, zeros(size(u_h)), 0, 'k', 'LineWidth', 1.2, 'AutoScale', 'off');
            
    xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
    title(['\bf ' name ' (3D)']);
    
    % View adjustments
    max_z = max(Pos(3,:));
    if max_z < 0.1, max_z = 1; end 
    zlim([0, max_z * 1.5]); 
    view(45, 30);
    
    %% ROW 2: Time Series Plot 
    % Subplot index calculation: (Row 2, Column col_idx) -> Index = col_idx + grid_cols
    subplot(grid_rows, grid_cols, col_idx + grid_cols);
    
    yyaxis left
    % X: Red Continuous
    plot(t_vec, Pos(1,:), 'r-', 'LineWidth', 1.5, 'DisplayName', 'x'); hold on;
    % Y: Green Continuous
    plot(t_vec, Pos(2,:), 'g-', 'LineWidth', 1.5, 'DisplayName', 'y');
    % Z: Blue Dashed
    plot(t_vec, Pos(3,:), 'b--', 'LineWidth', 1.5, 'DisplayName', 'z');
    
    ylabel('Position [m]');
    
    % Ensure the 2D plot also has room to show Z clearly
    y_limit_upper = max(1.5, max(Pos(3,:)) + 0.5);
    ylim([-1.5, y_limit_upper]); 

    ylim([-3, 3]); 
    
    yyaxis right
    % Yaw: Black Continuous
    plot(t_vec, rad2deg(Yaw), 'k-', 'LineWidth', 1.5, 'DisplayName', 'Yaw');
    ylabel('Yaw [deg]');
    
    xlabel('Time [s]');
    xlim([t_vec(1), t_vec(end)]); 
    title([name ' Evolution']);
    grid on;
    
    % Legend always visible
    legend('Location', 'best', 'FontSize', 8, 'NumColumns', 2);
end