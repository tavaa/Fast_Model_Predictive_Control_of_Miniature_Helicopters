% TERMINAL COST TESTS
% Unit Testing Suite for TerminalCost.m
%
% Purpose:
%   1. Dimension & Symmetry verification of the Cost-to-Go matrix P (Qf).
%   2. Positive Definiteness (Lyapunov function candidate requirement).
%   3. Riccati Equation Consistency (Verifies P satisfies the DARE).

clear; clc; close all;

fprintf('TESTING TERMINAL COST (DLQR) \n');

total_tests = 0;
passed_tests = 0;

try
    %% SETUP
    mp = ModelParams;
    model = HelicopterModel(false);
    
    % Standard weights
    Q = diag([10, 10, 20, 5, 1, 1, 1, 1, 0.1, 0.1]);
    R = diag([0.1, 0.1, 1, 0.1]);
    Ts = 0.1;

    %% TEST 1: Computation & Dimensions
    fprintf('[Test 1] Dimensions & Structure... ');
    
    Qf = TerminalCost.compute(Q, R, model, Ts);
    
    is_square = all(size(Qf) == [10, 10]);
    is_symmetric = norm(Qf - Qf') < 1e-8;
    
    if is_square && is_symmetric
        print_pass();
    else
        print_fail('Qf is not 10x10 or not symmetric');
    end

    %% TEST 2: Positive Definiteness
    fprintf('[Test 2] Positive Definiteness... ');
    
    % A valid Lyapunov candidate P must be Positive Definite (x'Px > 0)
    eigenvalues = eig(Qf);
    
    if all(eigenvalues > 0)
        print_pass();
    else
        print_fail('Qf has non-positive eigenvalues (Not PD)');
    end

    %% TEST 3: DARE Satisfaction
    fprintf('[Test 3] Riccati Equation Consistency... ');
    
    % Re-derive local linearized matrices to check the math
    u_hover = [0; 0; mp.g / mp.bz; 0];
    x_hover = zeros(10,1); x_hover(3) = 1.0;
    
    [Ac, Bc] = model.get_jacobians(x_hover, u_hover);
    sys_d = c2d(ss(Ac, Bc, eye(10), zeros(10,4)), Ts);
    A = sys_d.A; 
    B = sys_d.B;
    
    % Verify Algebraic Riccati Equation:
    % P = A'PA - (A'PB)(R + B'PB)^-1 (B'PA) + Q
    % Equivalent to: P = Q + A'PA - A'PB * K
    
    K_optimal = (R + B'*Qf*B) \ (B'*Qf*A);
    Qf_recalc = A'*Qf*A - (A'*Qf*B) * K_optimal + Q;
    
    residual = norm(Qf - Qf_recalc);
    
    if residual < 1e-6
        print_pass();
        fprintf('       -> Residual Error: %.2e (Numerical Precision)\n', residual);
    else
        print_fail(sprintf('DARE not satisfied. Residual: %.4f', residual));
    end

    %% TEST 4: Closed-Loop Stability
    fprintf('[Test 4] Closed-Loop Stability... ');
    
    % A_cl = A - B*K
    A_cl = A - B * K_optimal;
    e_cl = eig(A_cl);
    max_eig = max(abs(e_cl));
    
    if max_eig < 1.0
        print_pass();
        fprintf('       -> Max CL Eigenvalue: %.4f (Stable < 1)\n', max_eig);
    else
        print_fail(sprintf('Closed-loop system unstable. Max eig: %.4f', max_eig));
    end

    fprintf('Terminal Cost: %d/%d Tests Passed.\n', passed_tests, total_tests);

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