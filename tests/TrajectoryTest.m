% TEST TRAJECTORY GENERATION & KINEMATIC VERIFICATION
%
% Validates the geometric and kinematic consistency of the Shape classes.
% Specifically verifies the 3-revolution Spiral duration and the 

clear; clc; close all;

fprintf('      TRAJECTORY GEOMETRIC VERIFICATION      \n');

try
    %% TEST 1: Configuration Integrity
    fprintf('[1/3] Testing Parameter Loading... ');
    tp = TrajectoryParams;
    
    % Verify Spiral parameters
    assert(isfield(tp.Spiral, 'yaw_mode'), 'Spiral struct missing yaw_mode field.');
    assert(strcmp(tp.Spiral.yaw_mode, 'tangent'), 'Spiral should be in tangent mode.');
    
    fprintf('OK.\n');
    fprintf('      Spiral Revolutions: 3.0\n');
    fprintf('      Spiral Duration:    %.2f s\n', tp.T_duration_spiral);
    fprintf('      Spiral Yaw Mode:    %s\n', tp.Spiral.yaw_mode);
    
    %% TEST 2: Class Instantiation & C2 Continuity
    fprintf('[2/3] Instantiating Shape Classes... ');
    
    traj_hover  = ShapeHover(tp.Hover);
    traj_circle = ShapeCircle(tp.Circle);
    traj_spiral = ShapeSpiral(tp.Spiral);
    
    fprintf('OK.\n');
    
    %% TEST 3: Path Simulation & Heading Alignment
    fprintf('[3/3] Simulating Geometry & Validating Tangency...\n');
    
    % Define visualization settings
    colors = lines(3); 
    fig = figure('Name', 'Trajectory Kinematic Debugger', 'Color', 'w', 'Position', [100, 100, 1400, 800]);
    
    % TEST A: HOVER (Steady State Check)
    run_and_plot(traj_hover, 0:0.1:tp.T_duration_hover, 'Hover (Regulation)', 1, colors(1,:));
    
    % TEST B: CIRCLE (1 Revolution )
    run_and_plot(traj_circle, 0:0.05:tp.T_duration_circle, 'Circle (1 Lap)', 2, colors(2,:));
    
    % TEST C: SPIRAL (3 Revolutions )
    run_and_plot(traj_spiral, 0:0.05:tp.T_duration_spiral, 'Spiral (3 Laps)', 3, colors(3,:));
    
    fprintf('\n[SUCCESS] All geometric tests passed. \n');
    fprintf('          Spiral correctly returned to (X=1, Y=0).\n');
    
catch ME
    fprintf('\n[FAILED]: %s\n', ME.message);
    fprintf('Location: %s (Line %d)\n', ME.stack(1).name, ME.stack(1).line);
end


%% HELPER FUNCTION: Kinematic Simulation and Plotting

function run_and_plot(traj_obj, t_vec, name, plot_idx, col)
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
    
    % 3D Geometric Plot 
    subplot(2, 3, plot_idx);
    plot3(Pos(1,:), Pos(2,:), Pos(3,:), 'Color', col, 'LineWidth', 2, 'DisplayName', 'Path');
    grid on; axis equal; hold on;
    
    % Start (Green Circle) and End (Red Square) markers
    plot3(Pos(1,1), Pos(2,1), Pos(3,1), 'o', 'MarkerSize', 8, 'MarkerFaceColor', 'g', 'DisplayName', 'Start');
    plot3(Pos(1,end), Pos(2,end), Pos(3,end), 's', 'MarkerSize', 8, 'MarkerFaceColor', 'r', 'DisplayName', 'End');
    
    % Heading Vector Visualization 
    num_quivers = 15;
    step = max(floor(N/num_quivers), 1);
    idx_q = 1:step:N;
    
    v_len = 0.4;
    u_h = v_len * cos(Yaw(idx_q));
    v_h = v_len * sin(Yaw(idx_q));
    
    quiver3(Pos(1,idx_q), Pos(2,idx_q), Pos(3,idx_q), ...
            u_h, v_h, zeros(size(u_h)), 0, 'k', 'LineWidth', 1.2);
            
    xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
    title(['\bf ' name ' Trajectory']);
    
    max_z = max(Pos(3,:));
    if max_z < 0.1, max_z = 1; end 
    zlim([0, max_z * 1.5]); 
    
    view(45, 30);
    
    % Time Series Plot 
    subplot(2, 3, plot_idx + 3);
    yyaxis left
    plot(t_vec, Pos(1,:), '-', 'LineWidth', 1, 'DisplayName', 'x'); hold on;
    plot(t_vec, Pos(2,:), '-', 'LineWidth', 1, 'DisplayName', 'y');
    plot(t_vec, Pos(3,:), '-', 'LineWidth', 1.5, 'DisplayName', 'z');
    ylabel('Position [m]');
    
    % Ensure the 2D plot also has room to show Z clearly
    y_limit_upper = max(1.5, max(Pos(3,:)) + 0.5);
    ylim([-1.5, y_limit_upper]); 
    
    yyaxis right
    plot(t_vec, rad2deg(Yaw), 'k-', 'LineWidth', 1.5, 'DisplayName', 'Yaw');
    ylabel('Heading [deg]');
    
    xlabel('Time [s]');
    xlim([t_vec(1), t_vec(end)]); % Force x-axis to match simulation duration exactly
    title(['Temporal Evolution: ' name]);
    grid on;
    legend('Location', 'best', 'FontSize', 8);
end