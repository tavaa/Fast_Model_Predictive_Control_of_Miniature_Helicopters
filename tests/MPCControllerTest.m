% MPC CONTROLLER TEST
% Unit Testing Suite for MPCController.m
%
% Purpose:
%   1. Initialization (Cold Start & Flatness Mapping)
%   2. Solver Execution (Recursive Feasibility)
%   3. Regulation Accuracy (Hover stability)
%   4. Constraint Satisfaction (Input saturation)
%   5. Warm-Start Logic (Nominal trajectory shifting)

clear; clc; close all;

fprintf('TESTING MPC CONTROLLER PIPELINE \n');

total_tests = 0;
passed_tests = 0;

try
    %% SETUP
    mp = ModelParams;
    model = HelicopterModel(false); % Exact non-linear model
    mpc = MPCController(model);
    
    % Horizon length from params
    N = mpc.cp.p; 
    nx = 10; 
    nu = 4;
    
    %% TEST 1: Initialization (Cold Start)
    % init() must populate nominal_x/u using FlatnessMap
    fprintf('[Test 1] Initialization & Flatness Mapping... ');
    
    % Mock Trajectory: Static Hover at Origin
    % z = [x, y, z, psi]
    trajMock.get_flat_outputs = @(t) deal(zeros(4,1), zeros(4,1), zeros(4,1));
    
    x0 = zeros(10,1);
    t0 = 0;
    
    mpc.init(x0, trajMock, t0);
    
    % Check dimensions
    check_x = all(size(mpc.nominal_x) == [nx, N+1]);
    check_u = all(size(mpc.nominal_u) == [nu, N]);
    
    % Check content (Should be hovering inputs)
    u_hover_val = mp.g / mp.bz;
    check_val = abs(mpc.nominal_u(3,1) - u_hover_val) < 1e-5;
    
    if check_x && check_u && check_val
        print_pass();
    else
        print_fail('Nominal trajectory initialization failed');
    end

    %% TEST 2: Hover Stability (Zero Error)
    % If state=reference, Cost ~ 0 and u ~ u_hover
    fprintf('[Test 2] Hover Regulation (Steady State)... ');
    
    % References (Stay at origin)
    xref_seq = zeros(nx, N+1);
    uref_seq = zeros(nu, N);
    uref_seq(3,:) = u_hover_val; % Feedforward gravity comp
    
    [u_opt, info] = mpc.solve(x0, xref_seq, uref_seq);
    
    % Expect very low cost and u close to hover
    if info.cost < 1e-4 && abs(u_opt(3) - u_hover_val) < 1e-3
        print_pass();
    else
        print_fail(sprintf('Cost too high (%.4f) or Input wrong', info.cost));
    end

    %% TEST 3: Step Response (Active Control)
    % If state != reference, u_opt must deviate to correct error
    fprintf('[Test 3] Step Response Action... ');
    
    % Target: x = 1.0 (Forward)
    xref_step = zeros(nx, N+1);
    xref_step(1, :) = 1.0; 
    
    [u_step, ~] = mpc.solve(x0, xref_step, uref_seq);
    
    % To move forward X+, need Pitch negative (nose down)
    % Pitch is controlled by u1 (approx). 
    % Note: Pitch definition depends on frame, but u1 should NOT be 0.
    
    if abs(u_step(1)) > 1e-3
        print_pass();
    else
        print_fail('Controller ignored step reference (u_pitch ~ 0)');
    end

    %% TEST 4: Constraint Satisfaction
    % u_opt must stay within [min, max] even with aggressive ref
    fprintf('[Test 4] Input Constraint Enforcement... ');
    
    % Aggressive Target: 100m away
    xref_far = zeros(nx, N+1);
    xref_far(1, :) = 100.0;
    
    [u_sat, ~] = mpc.solve(x0, xref_far, uref_seq);
    
    % Check limits
    u_min = mp.u_min;
    u_max = mp.u_max;
    
    violation = any(u_sat < u_min - 1e-5) || any(u_sat > u_max + 1e-5);
    
    if ~violation
        print_pass();
    else
        print_fail('Input limits violated during aggressive maneuver');
        disp('Applied u:'); disp(u_sat');
        disp('Max u:'); disp(u_max');
    end

    %% TEST 5: Warm Start Logic
    % After solve(), nominal_x should shift left by 1
    fprintf('[Test 5] Warm Start Trajectory Shift... ');
    
    % Capture state before solve (from Test 4 context)
    prev_nominal = mpc.nominal_x; 
    
    % Run one more step
    mpc.solve(x0, xref_far, uref_seq);
    
    curr_nominal = mpc.nominal_x;
    
    % Logic: The new nominal(:,1) is updated with x_meas in solve().
    % The rest nominal(:,2:end) should roughly resemble prev_nominal(:,3:end) 
    % shifted, but updated by the solver dynamics.
    % A strict equality check isn't possible due to optimization updates,
    % but ensure the dimensions are preserved and data changed.
    
    if all(size(curr_nominal) == [nx, N+1]) && norm(curr_nominal - prev_nominal) > 1e-6
        print_pass();
    else
        print_fail('Nominal trajectory did not update/shift correctly');
    end
    
    fprintf('MPC Pipeline: %d/%d Tests Passed.\n', passed_tests, total_tests);

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