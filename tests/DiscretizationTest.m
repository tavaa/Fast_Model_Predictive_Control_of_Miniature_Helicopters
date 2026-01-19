% DISCRETIZATION TEST
% Unit Testing Suite for Discretization.m
%
% Verifies:
% 1. Identity/Zero handling.
% 2. Euler vs ZOH accuracy on a known Double Integrator system.
% 3. Stability check (Eigenvalues).

clear; clc;

fprintf('TESTING DISCRETIZATION \n');

total_tests = 0;
passed_tests = 0;

try
    %% TEST 1: Dimensions & Identity
    fprintf('[Test 1] Dimensions check... ');
    
    nx = 4; nu = 2;
    A = rand(nx, nx); B = rand(nx, nu);
    Ts = 0.1;
    
    [Ad, Bd] = Discretization.discretize(A, B, Ts, 'euler');
    
    if all(size(Ad) == [nx, nx]) && all(size(Bd) == [nx, nu])
        print_pass();
    else
        print_fail('Wrong output dimensions');
    end

    %% TEST 2: Double Integrator (Analytical Verification)
    % System: Mass sliding with Force input.
    % p_dot = v
    % v_dot = u/m (let m=1)
    %
    % A = [0 1; 0 0], B = [0; 1]
    
    fprintf('[Test 2] Double Integrator Accuracy (Ts=0.1s)... ');
    
    A_di = [0 1; 0 0];
    B_di = [0; 1];
    Ts = 0.1;
    
    % Analytical Exact Solution (ZOH) for Double Integrator:
    % Ad = [1 Ts; 0 1], Bd = [0.5*Ts^2; Ts]
    Ad_exact = [1, Ts; 0, 1];
    Bd_exact = [0.5*Ts^2; Ts];
    
    % Evaluate Euler 
    [Ad_eu, Bd_eu] = Discretization.discretize(A_di, B_di, Ts, 'euler');
    % Euler Theory: Ad = I+ATs = [1 Ts; 0 1] (Correct for A), Bd = BTs = [0; Ts] (Wrong for B!)
    
    % Evaluate ZOH 
    [Ad_zoh, Bd_zoh] = Discretization.discretize(A_di, B_di, Ts, 'zoh');
    
    % Errors
    err_zoh = norm(Ad_zoh - Ad_exact) + norm(Bd_zoh - Bd_exact);
    err_eu  = norm(Ad_eu - Ad_exact) + norm(Bd_eu - Bd_exact);
    
    % Logic: ZOH must be essentially 0 error. Euler will have error on position update from accel.
    if err_zoh < 1e-12 && err_eu > 1e-12
        print_pass();
        fprintf('       -> ZOH Error: %.2e (Perfect)\n', err_zoh);
        fprintf('       -> Euler Error: %.2e (Expected Approx)\n', err_eu);
    else
        print_fail('ZOH failed to match analytical solution');
    end

    %% TEST 3: Stability Limits 
    % System: Fast decay dx/dt = -100 * x
    % Time constant = 0.01s.
    % If we sample at Ts = 0.021s (Nyquist/Stability limit for Euler is 2/lambda = 0.02)
    
    fprintf('[Test 3] Stability on Stiff System (-100x)... ');
    
    A_stiff = -100; B_stiff = 0;
    Ts_large = 0.021; 
    
    [Ad_eu, ~]  = Discretization.discretize(A_stiff, B_stiff, Ts_large, 'euler');
    [Ad_zoh, ~] = Discretization.discretize(A_stiff, B_stiff, Ts_large, 'zoh');
    
    % Euler: 1 + (-100 * 0.021) = 1 - 2.1 = -1.1 (Magnitude > 1 -> UNSTABLE)
    % ZOH: exp(-100 * 0.021) = exp(-2.1) = 0.12 (Magnitude < 1 -> STABLE)
    
    if abs(Ad_eu) > 1 && abs(Ad_zoh) < 1
        print_pass();
        fprintf('       -> Euler Eigenvalue: %.2f (Unstable)\n', Ad_eu);
        fprintf('       -> ZOH Eigenvalue:   %.2f (Stable)\n', Ad_zoh);
    else
        print_fail('Stability check mismatch');
    end

    %% SUMMARY
    fprintf('Discretization Summary: %d/%d Tests Passed.\n', passed_tests, total_tests);

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