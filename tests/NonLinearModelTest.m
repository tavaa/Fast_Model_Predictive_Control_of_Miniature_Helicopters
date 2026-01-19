% NON LINEAR MODEL TEST
% Unit Test for HelicopterModel.m (Non-Linear Dynamics)
%
% Purpose:
%   1. Instantaneous Numeric Assertions (Derivative verification)
%   2. Dynamic Visual Validation (ODE Integration)
%
% Dependencies:
%   - ModelParams.m
%   - HelicopterModel.m

clear; clc; close all;
fprintf('TESTING NON-LINEAR HELICOPTER MODEL \n');

%% INITIALIZATION 
mp = ModelParams;

% Instantiate Model
% Full Physics = false implies exact non-linear math.
model = HelicopterModel(false);

% Test Counters
total_tests = 0;
passed_tests = 0;

try
    %% INSTANTANEOUS NUMERIC ASSERTIONS
    fprintf('PART 1: NUMERIC CHECKS \n');

    % TEST 1: Hover Equilibrium
    % f(x=0, u=hover) => dx/dt = 0 (No translational/rotational accel)

    fprintf('[Test 1] Hover Equilibrium ');
    x0 = zeros(10, 1);
    u_hover = [0; 0; mp.g / mp.bz; 0]; 
    dxdt = model.dynamics(0, x0, u_hover);
    
    % Assertion: Acceleration residuals < 1e-10
    if norm(dxdt(5:8)) < 1e-10, print_pass(); else, print_fail('Residual Accel'); end

    % TEST2: Free Fall (Gravity)
    % f(x=0, u=0) => z_accel = -g
    fprintf('[Test 2] Free Fall (Gravity)');
    u_zero = zeros(4,1);
    dxdt = model.dynamics(0, x0, u_zero);
    
    % Assertion: Vertical acceleration equals negative gravity
    if abs(dxdt(7) - (-mp.g)) < 1e-10, print_pass(); else, print_fail('Z Accel'); end

    % TEST 3: Kinematics (Rotation Matrix)
    % v_inertial = R(pitch=90) * v_body
    % If pitch=90deg and v_body_x=10, then v_inertial_z=10 (up)
    fprintf('[Test 3] Kinematics (Rotation Matrix) ');
    x_rot = zeros(10, 1); x_rot(4) = pi/2; x_rot(5) = 10.0;
    dxdt = model.dynamics(0, x_rot, u_hover);
    
    % Assertion: Check transformed velocity components
    if abs(dxdt(1)) < 1e-10 && abs(dxdt(2) - 10.0) < 1e-10, print_pass(); else, print_fail('Inertial Vel'); end

    % TEST 4: Aerodynamic Drag
    % drag_accel propto v_body
    fprintf('[Test 4] Aerodynamic Drag.');
    x_drag = zeros(10, 1); x_drag(5) = 10.0;
    dxdt = model.dynamics(0, x_drag, u_hover);
    expected_drag = mp.kx * 10.0;
    
    % Assertion: Acceleration matches calculated damping force
    if abs(dxdt(5) - expected_drag) < 1e-10, print_pass(); else, print_fail('Drag Accel'); end

    % TEST 5: Coriolis Coupling
    % a_coriolis = v_linear x w_angular
    % Input: Vy=5, YawRate=2 => Ax should differ from input
    fprintf('[Test 5] Coriolis Coupling (Pirouette) ');
    x_cor = zeros(10, 1); x_cor(6) = 5.0; x_cor(8) = 2.0;
    dxdt = model.dynamics(0, x_cor, u_hover);
    
    % Assertion: Check for cross-product acceleration term
    if abs(dxdt(5) - 10.0) < 1e-10, print_pass(); else, print_fail('Coriolis Accel'); end

    % TEST 6: Integral States
    % d(ErrorInt)/dt = Ki * Error
    fprintf('[Test 6] Integral States ');
    x_int = zeros(10,1); x_int(1) = 2.0; xref = zeros(10,1);
    dxdt = model.dynamics(0, x_int, u_hover, xref);
    
    % Assertion: Derivative of integral state matches error * gain
    if abs(dxdt(9) - mp.ki*2.0) < 1e-10, print_pass(); else, print_fail('Integral Deriv'); end
    
    fprintf('Model Dynamics: %d/%d Tests Passed.\n', passed_tests, total_tests);

    %% DYNAMIC VISUAL VALIDATION
    fprintf('\n PART 2: GENERATING PLOTS \n');
    
    fig = figure('Name', 'NonLinear Dynamics Check', 'Color', 'w', 'Position', [100, 100, 1200, 800]);
    
    % SIMULATION A: Free Fall & Drag Decay
    % Scenario: Initial Z=10m, Vx=5m/s. Motors OFF.
    % Expected: Parabolic Z drop, Exponential decay of Vx.
    tspan = [0 2];
    x0_sim = zeros(10,1); 
    x0_sim(3) = 10; % Altitude
    x0_sim(5) = 5;  % Forward Velocity
    
    [t_a, x_a] = ode45(@(t,x) model.dynamics(t, x, u_zero), tspan, x0_sim);

    % Plot A1: Z Position (Gravity verification)
    subplot(2,3,1);
    plot(t_a, x_a(:,3), 'b', 'LineWidth', 2); grid on;
    title('Free Fall: Z Position'); xlabel('t [s]'); ylabel('Z [m]');
    
    % Plot A2: Vx Velocity (Damping verification)
    subplot(2,3,4);
    plot(t_a, x_a(:,5), 'r', 'LineWidth', 2); grid on;
    title('Drag Test: Body Vx Decay'); xlabel('t [s]'); ylabel('Vx [m/s]');
    legend('Simulated (Exp Decay)');

    % SIMULATION B: Coriolis Effect
    % Scenario: Sideways motion (Vy) + Yaw Spin.
    % Expected: Emergence of Vx purely due to frame rotation.
    tspan = [0 2];
    x0_cor = zeros(10,1);
    x0_cor(6) = 2.0; % Vy
    x0_cor(8) = 3.0; % Yaw Rate
    
    [t_b, x_b] = ode45(@(t,x) model.dynamics(t, x, u_hover), tspan, x0_cor);
    
    % Plot B1: Coupled Velocities
    subplot(2,3,[2 5]);
    plot(t_b, x_b(:,5), 'r-', 'LineWidth', 2); hold on;
    plot(t_b, x_b(:,6), 'b--', 'LineWidth', 1.5);
    yline(0, 'k-', 'Alpha', 0.3);
    grid on;
    title({'Coriolis Coupling Validation', '(Input=0, but Vx changes!)'});
    xlabel('Time [s]'); ylabel('Body Velocity [m/s]');
    legend('Vx (Induced by Coupling)', 'Vy (Initial)');
    
    % Plot B2: Inertial Trajectory
    subplot(2,3,[3 6]);
    plot(x_b(:,1), x_b(:,2), 'k-o', 'MarkerIndices', 1:5:length(t_b));
    grid on; axis equal;
    title('Resulting Inertial Path (XY)');
    xlabel('X [m]'); ylabel('Y [m]');
    subtitle('Curved path due to Yaw+Velocity');
    
    % SIMULATION C: Active Control Response
    % Scenario: Step Input [Pitch=0.5, Thrust=110%].
    % Expected: Forward acceleration and Climb.
    tspan = [0 2];
    x0_move = zeros(10,1);
    u_move = [0.5; 0; (mp.g / mp.bz) * 1.1; 0];
    
    [t_c, x_c] = ode45(@(t,x) model.dynamics(t, x, u_move), tspan, x0_move);
    
    fig2 = figure('Name', 'Active Control Check', 'Color', 'w', 'Position', [150, 150, 1000, 500]);
    
    % Plot C1: Velocities
    subplot(1,2,1);
    plot(t_c, x_c(:,5), 'm-', 'LineWidth', 2); hold on; % Vx
    plot(t_c, x_c(:,7), 'c--', 'LineWidth', 2); % Vz
    grid on;
    title('C1. Motor Response (Const Input)');
    xlabel('Time [s]'); ylabel('Velocity [m/s]');
    legend('Vx (Pitch Resp)', 'Vz (Thrust Resp)', 'Location', 'Best');
    
    % Plot C2: Trajectory
    subplot(1,2,2);
    plot(x_c(:,1), x_c(:,3), 'm-s', 'LineWidth', 1.5, 'MarkerIndices', 1:5:length(t_c));
    grid on; 
    title('C2. Side View Trajectory (XZ)');
    xlabel('X Position [m]'); ylabel('Z Altitude [m]');
    subtitle('Should move Forward and Up');
    
    fprintf('Plots generated successfully.\n');

catch ME
    fprintf('\n\nCRITICAL ERROR: %s\n', ME.message);
    disp(ME.stack(1));
end

%% HELPER FUNCTIONS 

function print_pass()
    fprintf('[ PASS ]\n');
    % Update counters in caller workspace
    evalin('caller', 'passed_tests = passed_tests + 1;');
    evalin('caller', 'total_tests = total_tests + 1;');
end

function print_fail(msg)
    fprintf('[ FAIL ] -> %s\n', msg);
    evalin('caller', 'total_tests = total_tests + 1;');
end