% LTV LINEARIZATION TEST
% Unit Testing Suite for HelicopterModel.m (LTV Linearization)
%
% This script verifies the Analytical Jacobians (A, B) used by the MPC.
% It performs 3 types of tests:
% 1. STRUCTURAL CHECKS: Verify dimensions and sparsity patterns in Hover.
% 2. PAPER FIDELITY: Verify the "Model Identification" simplification logic.
% 3. MATHEMATICAL ACCURACY: Compare Analytical vs Finite Difference Jacobians.

clear; clc; close all;

fprintf('TESTING LTV LINEARIZATION (A, B) \n');

% Load Parameters
mp = ModelParams;

% Two instances to compare:
model_full = HelicopterModel(false); % Exact Math
model_sim = HelicopterModel(true);   % Paper Simplified (MPC Model)

% Test counters
total_tests = 0;
passed_tests = 0;

try
    %% PART 1: STRUCTURAL CHECKS (HOVER)
    fprintf('PART 1: STRUCTURAL CHECKS (HOVER) \n');
    
    % Setup Hover Condition
    x_hover = zeros(10, 1);
    u_hover = [0; 0; mp.g / mp.bz; 0];
    
    % LTV Linearization
    [A, B] = model_full.get_jacobians(x_hover, u_hover);
    
    % Dimensions
    fprintf('[Test 1.1] Matrix Dimensions ');
    if all(size(A)==[10 10]) && all(size(B)==[10 4])
        print_pass();
    else
        print_fail('Wrong Size');
    end
    
    % Matrix Gains (Diagonal block)
    fprintf('[Test 1.2] B Matrix Inputs Mapping ');
    % B(5,1) should be bx, B(6,2) by, etc.
    expected_gains = [mp.bx, mp.by, mp.bz, mp.bPsi];
    actual_gains   = [B(5,1), B(6,2), B(7,3), B(8,4)];
    
    if norm(actual_gains - expected_gains) < 1e-10
        print_pass();
    else
        print_fail('B Matrix coefficients mismatch');
    end
    
    % A Matrix Kinematics (Identity at Psi=0)
    fprintf('[Test 1.3] Kinematics at Psi=0. ');
    % dxI = dxB (Row 1, Col 5 should be 1 -> cos 0)
    if abs(A(1,5) - 1.0) < 1e-10 && abs(A(2,6) - 1.0) < 1e-10
        print_pass();
    else
        print_fail('Rotation Block incorrect');
    end
    
    % Integral Action Connections
    fprintf('[Test 1.4] Integral States Structure ');
    % d(x_int)/dxI should be ki (Row 9, Col 1)
    if abs(A(9,1) - mp.ki) < 1e-10 && abs(A(10,2) - mp.ki) < 1e-10
        print_pass();
    else
        print_fail('Integral terms missing or wrong');
    end


    %% PART 2: PAPER FIDELITY (SIMPLIFICATION FLAG)
    fprintf('\n PART 2: PAPER FIDELITY (COUPLING REMOVAL) \n');
    
    % Setup Stress Condition 
    % High Lateral Velocity + High Yaw Rate
    x_stress = zeros(10, 1);
    x_stress(6) = 5.0; % dyB
    x_stress(8) = 2.0; % dPsi
    
    [A_full, ~] = model_full.get_jacobians(x_stress, u_hover);
    [A_simp, ~] = model_sim.get_jacobians(x_stress, u_hover);
    
    % A_full (visual check)
    % A_simp
    
    % Full Model Cross-Terms
    fprintf('[Test 2.1] Full Model contains Coriolis ');
    % In full model, A(5,6) should be dPsi (2.0)
    % Eq: ddxB = ... + dPsi * dyB
    % Jacobian d(ddxB)/d(dyB) = dPsi
    if abs(A_full(5,6) - x_stress(8)) < 1e-10
        print_pass();
    else
        print_fail(sprintf('Expected %.1f, got %.1f', x_stress(8), A_full(5,6)));
    end
    
    % Simplified Model Removes Terms
    fprintf('[Test 2.2] Simplified Model ZEROES Coriolis ');
    if A_simp(5,6) == 0 && A_simp(6,5) == 0
        print_pass();
    else
        print_fail('Coupling terms NOT removed!');
    end

    %% PART 3: MATHEMATICAL VALIDATION (NUMERICAL DIFF)
    fprintf('\n PART 3: NUMERICAL VALIDATION (FINITE DIFF) \n');
    % Verify that the Full Analytical Jacobian matches the Numerical Gradient
    % of the non-linear dynamics function.
    
    % Random state to activate all sines/cosines/couplings
    x_rand = rand(10, 1); 
    u_rand = rand(4, 1);
    
    % Analytical
    [A_ana, B_ana] = model_full.get_jacobians(x_rand, u_rand);
    
    % Numerical (Finite Differences)
    epsilon = 1e-6;
    [A_num, B_num] = get_numerical_jacobians(model_full, x_rand, u_rand, epsilon);
    
    % Matrix A Accuracy
    fprintf('[Test 3.1] Matrix A Accuracy vs Numerical ');
    err_A = norm(A_ana - A_num, 'fro'); % Frobenius norm
    if err_A < 1e-5
        print_pass();
    else
        print_fail(sprintf('Error Norm: %.2e', err_A));
        % Debug hint
        [r, c] = find(abs(A_ana - A_num) > 1e-4);
        if ~isempty(r)
            fprintf('       -> Mismatch at indices: \n');
            for i=1:min(length(r),3), fprintf('          (%d,%d) Ana:%.2f Num:%.2f\n', r(i),c(i), A_ana(r(i),c(i)), A_num(r(i),c(i))); end
        end
    end
    
    % Matrix B Accuracy
    fprintf('[Test 3.2] Matrix B Accuracy vs Numerical ');
    err_B = norm(B_ana - B_num, 'fro');
    if err_B < 1e-5
        print_pass();
    else
        print_fail(sprintf('Error Norm: %.2e', err_B));
    end

    %% SUMMARY
    fprintf('\n Linearization Summary: %d/%d Tests Passed.\n', passed_tests, total_tests);

catch ME
    fprintf('\n\nCRITICAL ERROR: %s\n', ME.message);
    disp(ME.stack(1));
end

%% HELPER FUNCTIONS

function [A_num, B_num] = get_numerical_jacobians(model, x, u, eps)
    nx = length(x);
    nu = length(u);
    
    % Base dynamics
    f0 = model.dynamics(0, x, u);
    
    A_num = zeros(nx, nx);
    B_num = zeros(nx, nu);
    
    % Compute A = df/dx
    for i = 1:nx
        x_pert = x;
        x_pert(i) = x_pert(i) + eps;
        f_pert = model.dynamics(0, x_pert, u);
        A_num(:, i) = (f_pert - f0) / eps;
    end
    
    % Compute B = df/du
    for i = 1:nu
        u_pert = u;
        u_pert(i) = u_pert(i) + eps;
        f_pert = model.dynamics(0, x, u_pert);
        B_num(:, i) = (f_pert - f0) / eps;
    end
end

function print_pass()
    fprintf('[ PASS ]\n');
    evalin('caller', 'passed_tests = passed_tests + 1;');
    evalin('caller', 'total_tests = total_tests + 1;');
end

function print_fail(msg)
    fprintf('[ FAIL ] -> %s\n', msg);
    evalin('caller', 'total_tests = total_tests + 1;');
end