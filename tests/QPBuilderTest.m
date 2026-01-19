% QP BUILDER TEST
% Unit Testing for QPBuilder.m
%
% This script verifies the construction of the Sparse QP matrices.
% It focuses on:
% 1. DIMENSIONS: Correct size based on Horizon (N), states (nx), inputs (nu).
% 2. STRUCTURE: Block-diagonal Hessian and Staircase Dynamics (LTV).
% 3. SPARSITY: Verifying that matrices are actually sparse (RAM efficiency).
% 4. VALUES: Checking if Weights and Constraints are placed correctly.

clear; clc; close all;

fprintf('TESTING QP BUILDER (SPARSE FORMULATION) \n');

total_tests = 0;
passed_tests = 0;

try
    %% SETUP
    % System: 2 States, 1 Input, Horizon N=3
    nx = 2; 
    nu = 1; 
    N  = 3;
    
    % Weights
    Q  = diag([10, 20]);      % State penalty
    R  = diag([1]);           % Input penalty
    Qf = diag([100, 200]);    % Terminal penalty
    
    % LTV Model 
    Ad_seq = cell(1, N);
    Bd_seq = cell(1, N);
    d_seq  = cell(1, N);
    
    for k = 1:N
        % LTV: Dynamics change slightly at each step
        Ad_seq{k} = eye(nx) * k;      % A_k scales with k (just for testing)
        Bd_seq{k} = ones(nx, nu) * k; % B_k scales with k
        d_seq{k}  = [0.1; 0.2] * k;   % Affine term
    end

    %Ad_seq
    %Bd_seq
    %d_seq
    
    % References
    x0_meas  = [5; 5]; % Initial condition
    xref_seq = zeros(nx, N+1); % Target: Origin
    uref_seq = zeros(nu, N);   % Target: Zero input
    
    % Limits
    u_lims.min = -1; u_lims.max = 1;
    x_lims = []; % No state constraints for now
    
    % BUILD QP
    fprintf('[INFO] Building QP matrices for N=%d, nx=%d, nu=%d...\n', N, nx, nu);
    [H, f, Aeq, beq, lb, ub] = QPBuilder.build(Ad_seq, Bd_seq, d_seq, ...
                                              x0_meas, xref_seq, uref_seq, ...
                                              Q, R, Qf, u_lims, x_lims);
                                          
    %% TEST 1: Dimensions Check
    fprintf('[Test 1] Matrix Dimensions... ');
    
    % Expected Variables Z: [x0, u0, x1, u1, x2, u2, x3]
    % Count: (N+1)*nx + N*nu = 4*2 + 3*1 = 8 + 3 = 11 variables
    nz_expected = (N+1)*nx + N*nu;
    
    % Expected Equality Constraints: (N+1)*nx = 4*2 = 8 equations
    % (1 initial condition + N dynamic steps)
    neq_expected = (N+1)*nx;
    
    if size(H,1) == nz_expected && size(H,2) == nz_expected && ...
       size(Aeq,1) == neq_expected && size(Aeq,2) == nz_expected && ...
       length(f) == nz_expected && length(beq) == neq_expected
        print_pass();
    else
        print_fail(sprintf('Got H[%d,%d], Aeq[%d,%d]. Expected H[%d,%d], Aeq[%d,%d]', ...
            size(H,1), size(H,2), size(Aeq,1), size(Aeq,2), ...
            nz_expected, nz_expected, neq_expected, nz_expected));
    end

    %% TEST 2: Sparsity Check
    fprintf('[Test 2] Sparsity (Is Sparse?)... ');
    if issparse(H) && issparse(Aeq)
        print_pass();
        fprintf('       -> H non-zeros: %d (%.1f%% full)\n', nnz(H), 100*nnz(H)/numel(H));
        fprintf('       -> Aeq non-zeros: %d (%.1f%% full)\n', nnz(Aeq), 100*nnz(Aeq)/numel(Aeq));
    else
        print_fail('Matrices are DENSE (Full). Should be SPARSE.');
    end

    %H
    %Aeq

    %% TEST 3: Hessian Structure (Block Diagonal)
    fprintf('[Test 3] Hessian Structure (Cost Function)... ');
    
    % The last block should be Qf
    % Z structure ends with xN. So bottom-right nx*nx block of H is Qf.
    H_full = full(H); % Convert to full for easy indexing in test
    Qf_extracted = H_full(end-nx+1:end, end-nx+1:end);
    
    % The first block should be Q
    Q_extracted = H_full(1:nx, 1:nx);
    
    % The input block u0 (indices nx+1 : nx+nu) should be R
    R_extracted = H_full(nx+1:nx+nu, nx+1:nx+nu);
    
    if isequal(Qf_extracted, Qf) && isequal(Q_extracted, Q) && isequal(R_extracted, R)
        print_pass();
    else
        print_fail('Hessian blocks do not match weights Q, R, Qf');
    end

    %% TEST 4: Dynamics Constraints (LTV Structure)
    fprintf('[Test 4] Equality Constraints (Dynamics)... ');
    
    % Row 1 to nx: Initial Condition x0 = x0_meas
    % Aeq(1:nx, 1:nx) should be Identity
    % beq(1:nx) should be x0_meas
    
    Aeq_full = full(Aeq);
    
    Init_Block = Aeq_full(1:nx, 1:nx);
    RHS_Init   = beq(1:nx);
    
    % Row nx+1 onwards: Dynamics x1 - B0u0 - A0x0 = d0
    % Aeq rows for step k=1 correspond to:
    % Coeff of x0: -A0
    % Coeff of u0: -B0
    % Coeff of x1:  I
    
    % Verify for k=1 (Horizon step 1)
    % A0 was defined as eye(nx)*1
    % B0 was defined as ones(nx,nu)*1
    
    % Indices in Z: x0 (1:2), u0 (3), x1 (4:5)
    row_start = nx + 1;
    row_end   = nx + nx;
    
    Block_x0 = Aeq_full(row_start:row_end, 1:2); % Should be -A0
    Block_u0 = Aeq_full(row_start:row_end, 3);   % Should be -B0
    Block_x1 = Aeq_full(row_start:row_end, 4:5); % Should be I
    
    check_init = isequal(Init_Block, eye(nx)) && isequal(RHS_Init, x0_meas);
    check_dyn  = isequal(Block_x0, -Ad_seq{1}) && ...
                 isequal(Block_u0, -Bd_seq{1}) && ...
                 isequal(Block_x1, eye(nx));
             
    if check_init && check_dyn
        print_pass();
    else
        print_fail('Dynamics constraints are malformed');
        disp('Block x0 (Expected -I):'); disp(Block_x0);
    end

    %% TEST 5: Bounds Check
    fprintf('[Test 5] Inequality Constraints (Bounds)... ');
    
    % Check u0 bounds. Index in Z is nx+1 = 3
    idx_u0 = 3;
    lb_u0 = lb(idx_u0);
    ub_u0 = ub(idx_u0);
    
    % Check x0 bounds (should be -inf / inf)
    idx_x0 = 1;
    
    if lb_u0 == -1 && ub_u0 == 1 && lb(idx_x0) == -inf
        print_pass();
    else
        print_fail('Bounds lb/ub incorrect');
    end

    %% VISUALIZATION (Spy Plot)
    fprintf('\nGenerating Sparsity Plot (Check Figure window)...\n');
    figure('Name', 'QP Matrix Structure', 'Color', 'w', 'Position', [100 100 1000 500]);
    
    subplot(1, 2, 1);
    spy(H);
    title(sprintf('Hessian H (Size: %dx%d)', size(H)));
    xlabel('Decision Variables Z');
    
    subplot(1, 2, 2);
    spy(Aeq);
    title(sprintf('Constraints Aeq (Size: %dx%d)', size(Aeq)));
    xlabel('Decision Variables Z');
    ylabel('Constraints');
    
    sgtitle('Sparsity Pattern of MPC Matrices');

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