% FLATNESS MAP TEST
% Unit Testing Suite for FlatnessMap.m
%
% This script verifies the Differential Flatness Mapping (Inverse Dynamics).
% It checks if the calculated Inputs (u) and States (x) actually generate
% the desired Trajectory (z, dz, ddz) when applied to the Forward Model.
%
% Tests performed:
% 1. STATIC CONSISTENCY: Hover inputs check against gravity.
% 2. KINEMATIC INVERSION: Velocity rotation check.
% 3. DYNAMIC CONSISTENCY (Closed Loop):
%    Trajectory -> FlatnessMap -> (x_ref, u_ref) -> ForwardModel -> dxdt -> 
%    ... Calculate resulting Inertial Accel ... -> Compare with original ddz.

clear; clc; close all;

fprintf('TESTING FLATNESS MAP (INVERSE DYNAMICS) \n');

% Load Params and Models
mp = ModelParams;

% FULL physics model 
model = HelicopterModel(false); 

% Tests counters
total_tests = 0;
passed_tests = 0;

try
    %% TEST 1: Hover (Static Check)
    fprintf('[Test 1] Static Hover Mapping');
    
    % Desired: Pos=[0,0,1], Yaw=0, Vel=0, Acc=0
    z   = [0; 0; 1; 0];
    dz  = zeros(4,1);
    ddz = zeros(4,1);
    
    % compute ref using FlatnessMap
    [xref, uref] = FlatnessMap.map(z, dz, ddz);
    
    % Check 1: Reference velocity states should be 0
    if norm(xref(5:8)) < 1e-10
        % Check 2: Thrust should be exactly gravity compensation
        % uz = g / bz
        expected_uz = mp.g / mp.bz;
        if abs(uref(3) - expected_uz) < 1e-10
            print_pass();
        else
            print_fail(sprintf('Thrust %.2f != Expected %.2f', uref(3), expected_uz));
        end
    else
        print_fail('Non-zero velocities in hover ref');
    end
    
    %% TEST 2: Kinematics (Rotation Check)
    fprintf('[Test 2] Kinematic Inversion (90 deg Yaw)');
    
    % Desired: Moving NORTH (Y+) at 10m/s, facing WEST (Yaw=90 deg)
    z = [0; 0; 0; pi/2];
    
    % dz = [vx_I, vy_I, vz_I, dPsi] = [0, 10, 0, 0]
    dz = [0; 10; 0; 0]; %10 m/s west 
    ddz = zeros(4,1);
    
    % compute ref using FlatnessMap
    [xref, ~] = FlatnessMap.map(z, dz, ddz);
    % xref =[0 0 0 1.5708 10.0000 0 0 0 0 0]'
    % xref =[x_i y_i z_i psi xb_dot yb_dot zb_dot dpsi x_int y_int]'
     
    %dxB, dyB
    dxB_ref = xref(5);
    dyB_ref = xref(6);
    
    % Moving in direction of nose => dxB=10, dyB=0
    if abs(dxB_ref - 10.0) < 1e-10 && abs(dyB_ref) < 1e-10
        print_pass();
    else
        print_fail(sprintf('Body Vel [%.2f, %.2f] != Expected [10, 0]', dxB_ref, dyB_ref));
    end

    %% TEST 3: Dynamic Consistency 
    fprintf('[Test 3] Full Dynamic Consistency (Spiral)');
    
    % Generate a complex target point (e.g. Spiral at t=3.5s)
    % This includes Position, Velocity, Acceleration, Yaw, YawRate, YawAccel
    tp = TrajectoryParams;
    spiral = ShapeSpiral(tp.Spiral);
    t_test = 3.5;
    [z_des, dz_des, ddz_des] = spiral.get_flat_outputs(t_test);

    % DEBUG
    % z_des
    % dz_des
    % ddz_des
    
    % INVERSE: Map to x, u
    [xref, uref] = FlatnessMap.map(z_des, dz_des, ddz_des);

    % DEBUG
    % xref
    % uref
    
    % Apply x, u to Model
    % dxdt_model = f(xref, uref)
    dxdt = model.dynamics(0, xref, uref);
    % DEBUG 
    % dxdt
    
    % COMPARE ACCELERATIONS
    % The model returns derivatives of BODY velocities (ddxB, ddyB, ddzB)
    % The desired trajectory gave INERTIAL accelerations (ddxI, ddyI, ddzI)
    % convert Model Output -> Inertial Frame to compare.
    
    % Unpack State
    Psi = xref(4);
    dPsi = xref(8);
    dxB = xref(5); dyB = xref(6);
    
    % Unpack Model Derivatives (Body Accels)
    acc_B = dxdt(5:7); % [ddxB; ddyB; ddzB]
    
    % Inertial Acceleration Formula:
    % Simply differentiate the kinematic components:
    % d(dxI)/dt = d(cos*dxB - sin*dyB)/dt (Leibniz Rule)
    %           = (-sin*dPsi*dxB + cos*ddxB) - (cos*dPsi*dyB + sin*ddyB)
    
    sin_p = sin(Psi); cos_p = cos(Psi);
    
    % Calculation from extracted values
    ddxI_model = (cos_p*acc_B(1) - sin_p*acc_B(2)) + dPsi * (-sin_p*dxB - cos_p*dyB);
    ddyI_model = (sin_p*acc_B(1) + cos_p*acc_B(2)) + dPsi * ( cos_p*dxB - sin_p*dyB);
    ddzI_model = acc_B(3); % Vertical is decoupled
    
    acc_I_model = [ddxI_model; ddyI_model; ddzI_model];
    
    % Compare with ddz_des _> from get_flat_outputs (First 3 elements)
    err_acc = norm(acc_I_model - ddz_des(1:3));
    
    % Check Yaw Acceleration
    ddPsi_model = dxdt(8);
    err_yaw = abs(ddPsi_model - ddz_des(4));
    
    if err_acc < 1e-8 && err_yaw < 1e-8
        print_pass();
    else
        print_fail(sprintf('Accel Mismatch. ErrLin: %.2e, ErrYaw: %.2e', err_acc, err_yaw));
        fprintf('       Target Acc: [%.2f %.2f %.2f]\n', ddz_des(1:3));
        fprintf('       Model  Acc: [%.2f %.2f %.2f]\n', acc_I_model);
    end

    %% SUMMARY
    fprintf('Flatness Map Summary: %d/%d Tests Passed.\n', passed_tests, total_tests);

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