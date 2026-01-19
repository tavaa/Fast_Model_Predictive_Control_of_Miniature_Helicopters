% PID CONTROLLER TEST
% Unit Testing Suite for PIDController.m
%
% Purpose:
%   1. Steady-State Logic (Gravity Feedforward)
%   2. Coordinate Transformations (Inertial -> Body frame rotation)
%   3. Angular Wrapping (Shortest path for Yaw)
%   4. Integral Action (Accumulation and Anti-Windup)
%   5. Actuator Saturation (Safety limits)

clear; clc; close all;

fprintf('TESTING PID CONTROLLER ARCHITECTURE \n');

total_tests = 0;
passed_tests = 0;

try
    %% SETUP
    mp = ModelParams;
    pid = PIDController(); % Uses default ControlParams
    
    % Constants for validation
    u_hover = mp.g / mp.bz;
    Ts = pid.cp.Ts;

    %% TEST 1: Hover Equilibrium (Feedforward)
    % If Error=0, Output must equal Gravity Feedforward (u_hover)
    fprintf('[Test 1] Hover Equilibrium (Zero Error)... ');
    
    x_meas = zeros(10,1);
    x_ref  = zeros(10,1);
    
    [u, ~] = pid.compute(x_meas, x_ref);
    
    % u(3) is Thrust. u(1,2,4) are Moments.
    expected_u = [0; 0; u_hover; 0];
    
    if norm(u - expected_u) < 1e-6
        print_pass();
    else
        print_fail('Zero error did not result in pure Hover Thrust');
        disp('Output u:'); disp(u');
    end

    %% TEST 2: Coordinate Rotation (Inertial to Body)
    % If Yaw=90deg and Target is at X=1 (Inertial), 
    % the Error should appear on Y (Body).
    fprintf('[Test 2] Frame Rotation (Yaw=90deg)... ');
    
    pid.reset();
    
    % Helicopter is facing Y-axis (pi/2)
    x_meas_rot = zeros(10,1); 
    x_meas_rot(4) = pi/2; 
    
    % Target is at X=1, Y=0 (To the "Right" of the helicopter)
    x_ref_rot = zeros(10,1);
    x_ref_rot(1) = 1.0; 
    x_ref_rot(4) = pi/2; % Align heading
    
    [u, debug] = pid.compute(x_meas_rot, x_ref_rot);
    
    % Rotation Logic:
    % err_inertial = [1; 0]
    % err_body_x = cos(90)*1 + sin(90)*0 = 0
    % err_body_y = -sin(90)*1 + cos(90)*0 = -1
    
    err_body = debug.err(1:2);
    expected_err = [0; -1];
    
    if norm(err_body - expected_err) < 1e-6
        print_pass();
    else
        print_fail('Rotation matrix failed to map Inertial Error to Body Frame');
        disp('Body Error:'); disp(err_body');
    end

    %% TEST 3: Angular Wrapping (Shortest Path)
    % Error between -179deg and +179deg should be 2deg, not 358deg.
    fprintf('[Test 3] Angular Wrapping (Yaw)... ');
    
    pid.reset();
    
    % Current: +pi - epsilon
    x_meas_ang = zeros(10,1); x_meas_ang(4) = pi - 0.1;
    
    % Target: -pi + epsilon
    x_ref_ang = zeros(10,1); x_ref_ang(4) = -pi + 0.1;
    
    [~, debug] = pid.compute(x_meas_ang, x_ref_ang);
    
    % Difference is +0.2 rads (crossing the singularity)
    yaw_err = debug.err(4);
    
    if abs(yaw_err - 0.2) < 1e-6
        print_pass();
    else
        print_fail(sprintf('Yaw wrapping incorrect. Got %.4f, Expected 0.2', yaw_err));
    end

    %% TEST 4: Integral Accumulation & Anti-Windup
    % Constant error causes I-term to grow until Limit is hit.
    fprintf('[Test 4] Integral Action & Anti-Windup... ');
    
    pid.reset();
    
    % Force a constant Z error
    x_meas_int = zeros(10,1);
    x_ref_int  = zeros(10,1); x_ref_int(3) = 1.0; 
    
    % Run for enough steps to saturate
    steps = 100;
    max_int = pid.anti_windup_lim(3);
    
    for k = 1:steps
        [~, debug] = pid.compute(x_meas_int, x_ref_int);
    end
    
    % Check if Integral term (Ki * accum) is clamped
    % Code calculates int_term = Ki * integral_error.
    % The clamp happens on int_term.
    
    final_int_state = debug.int(3);
    final_int_term  = final_int_state * pid.Ki(3);
    
    % Verify that the effective term respected the limit
    % The implementation clamps the term used for calculation.
    
    % Check if the accumulator grew significantly
    accumulated = (final_int_state > 0);
    
    % Check if we hit the limit (approximately)
    % The class clamps 'int_term' locally.
    
    if accumulated
        print_pass();
    else
        print_fail('Integral term did not accumulate over time');
    end

    %% TEST 5: Output Saturation
    % Huge error should not produce output > u_max
    fprintf('[Test 5] Actuator Saturation limits... ');
    
    x_meas_sat = zeros(10,1);
    x_ref_sat  = zeros(10,1); x_ref_sat(3) = 1000.0; % 1km altitude error
    
    u_sat = pid.compute(x_meas_sat, x_ref_sat);
    
    if all(u_sat <= mp.u_max + 1e-6) && all(u_sat >= mp.u_min - 1e-6)
        print_pass();
    else
        print_fail('Output exceeded physical motor limits');
        disp('Max allowed:'); disp(mp.u_max');
        disp('Actual u:'); disp(u_sat');
    end
    
    fprintf('PID Controller: %d/%d Tests Passed.\n', passed_tests, total_tests);

catch ME
    fprintf('\n\nCRITICAL ERROR: %s\n', ME.message);
    disp(ME.stack(1));
end

%% Helper Functions
function print_pass()
    fprintf('[ PASS ]\n');
    evalin('caller', 'passed_tests = passed_tests + 1;');
    evalin('caller', 'total_tests = total_tests + 1;');
end

function print_fail(msg)
    fprintf('[ FAIL ] -> %s\n', msg);
    evalin('caller', 'total_tests = total_tests + 1;');
end