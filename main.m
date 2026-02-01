% Project Orchestrator & Pipeline Entry Point
% "Fast Model Predictive Control of Miniature Helicopters"
%
% OVERVIEW:
%   This script serves as the central entry point for the simulation and 
%   testing framework. It automatically configures the MATLAB environment 
%   (path management) and provides a hierarchical, interactive command-line 
%   interface (CLI) to execute full-system simulations or individual 
%   component validation tests.
%
% DIRECTORY STRUCTURE:
%   root/
%    +-- config/      
%    +-- models/      
%    +-- src/        
%    +-- scripts/     
%    +-- tests/      
%    +-- trajectory/  
%
% MODES OF OPERATION:
%   1. MPC Simulation Loop: 
%      Runs the complete Receding Horizon Control loop using the Linear 
%      Time-Varying (LTV) formulation, solving a QP at every time step.
%
%   2. PID Simulation Loop:
%      Runs a benchmark simulation using a classical PID 
%      architecture for performance comparison.
%
%   3. MPC Simulation Loop for Ts:
%        Runs in depth Simulation Loop for MPC, analyzing Ts 
%         and different initial conditions.
%
%   4. Unit Test Suite:
%      Access to a submenu for validating specific mathematical subsystems
%      (e.g., Physics, Linearization, Discretization, Optimization construction).
%
% REQUIREMENTS:
%   - Optimization Toolbox (for 'quadprog')
%   - Control System Toolbox (for 'dlqr' and 'ss')

% Initial cleanup
clear; clc; close all;

%% PATH SETUP (Run Once)
projectRoot = fileparts(mfilename('fullpath'));

% Add all necessary subfolders using absolute paths to ensure visibility
addpath(fullfile(projectRoot, 'config'));
addpath(fullfile(projectRoot, 'models'));
addpath(fullfile(projectRoot, 'trajectory'));
addpath(fullfile(projectRoot, 'scripts'));
addpath(fullfile(projectRoot, 'tests'));
addpath(fullfile(projectRoot, 'src'));

fprintf('Environment configured. Root: %s\n', projectRoot);
pause(1); 

%% MAIN PIPELINE LOOP
while true
    clc; 
    % ASCII HEADER 
    fprintf('\n');
    fprintf('  ==============================================================\n');
    fprintf('   ___ _   ___ _____   __  __ ___  ___ \n');
    fprintf('  | __/_\\ / __|_   _| |  \\/  | _ \\/ __|\n');
    fprintf('  | _/ _ \\\\__ \\ | |   | |\\/| |  _/ (__ \n');
    fprintf('  |_/_/ \\_\\___/ |_|   |_|  |_|_|  \\___|\n');
    fprintf('\n');
    fprintf('      MINIATURE HELICOPTER CONTROL FRAMEWORK\n');
    fprintf('  ==============================================================\n');
    fprintf('\n');
    
    fprintf('Select an operation mode:\n\n');

    fprintf('   [1] Run MPC Simulation Loop\n');
    fprintf('       (Check results/MPC/ for the results.)\n\n');

    fprintf('   [2] Run PID Simulation Loop\n');
    fprintf('       (Check results/PID/ for the results.)\n\n');

    fprintf('   [3] Run MPC Simulation Loop for Ts\n');
    fprintf('       (Tests in depth MPC Analysis based on Ts and Initialization strategies. )\n\n');

    fprintf('   [4] Enter Unit Test Suite\n');
    fprintf('       (Validate individual components: Model, QP, Math, etc.)\n\n');

    fprintf('   [0] Exit\n');
    fprintf('----------------------------------------------------------------\n');

    %% MAIN MENU INPUT
    try
        choice = input('Enter your choice [0-3]: ');
    catch
        choice = -1;
    end

    fprintf('\n');

    switch choice
        case 1
            % MPC SIMULATION 
            if exist('MPCSimulationLoop.m', 'file')
                fprintf('Starting MPC Simulation...\n');
                run('MPCSimulationLoop.m');
                waitForUser('Simulation Complete.');
            else
                fprintf('[ERROR] scripts/MPCSimulationLoop.m not found.\n');
                waitForUser('Error.');
            end

        case 2
            % PID SIMULATION 
            if exist('PIDSimulationLoop.m', 'file')
                fprintf('Starting PID Simulation...\n');
                run('PIDSimulationLoop.m');
                waitForUser('Simulation Complete.');
            else
                fprintf('[ERROR] scripts/PIDSimulationLoop.m not found.\n');
                waitForUser('Error.');
            end
        
        case 3
            % MPC Simulation Loop Ts
            if exist('MPCSimulationLoopTs.m', 'file')
                fprintf('Starting MPC Simulation with Ts...\n');
                run('MPCSimulationLoopTs.m');
                waitForUser('Simulation Complete.');
            else
                fprintf('[ERROR] scripts/MPCSimulationLoopTs.m not found.\n');
                waitForUser('Error.');
            end

        case 4
            % UNIT TEST SUITE SUB-LOOP 
            runTestSuite();

        case 0
            fprintf('Exiting Application.\n');
            break; 

        otherwise
            fprintf('Invalid choice. Please try again.\n');
            pause(1.5);
    end
end

fprintf('Done.\n');

%% SUB-MENU: UNIT TEST SUITE
function runTestSuite()
    while true
        clc;
        fprintf('========================================================\n');
        fprintf('      UNIT TEST SUITE                                   \n');
        fprintf('========================================================\n');
        
        fprintf('   [1] Trajectory Generation Test\n');
        fprintf('   [2] Non-Linear Model Physics Test\n');
        fprintf('   [3] LTV Linearization Test\n');
        fprintf('   [4] Flatness Map Test\n');
        fprintf('   [5] Discretization Test\n');
        fprintf('   [6] QP Builder Test\n');
        fprintf('   [7] MPC Controller Test\n');
        fprintf('   [8] Terminal Cost (DLQR) Test\n');
        fprintf('   [9] PID Controller Test\n');
        fprintf('\n   [0] Return to Main Pipeline\n');
        fprintf('--------------------------------------------------------\n');

        try
            t_choice = input('Select Test [0-9]: ');
        catch
            t_choice = -1;
        end
        
        fprintf('\n--------------------------------------------------------\n');

        if t_choice == 0
            return; % Exit sub-loop
        end

        switch t_choice
            case 1, run('TrajectoryTest.m');
            case 2, run('NonLinearModelTest.m');
            case 3, run('LTVLinearizationTest.m');
            case 4, run('FlatnessMapTest.m');
            case 5, run('DiscretizationTest.m');
            case 6, run('QPBuilderTest.m');
            case 7, run('MPCControllerTest.m');
            case 8, run('TerminalCostTest.m');
            case 9, run('PIDControllerTest.m');
            otherwise
                fprintf('Invalid selection.\n');
        end
        
        waitForUser('Test Finished.');
    end
end

%% HELPER FUNCTION
function waitForUser(msg)
    fprintf('\n--------------------------------------------------------\n');
    fprintf('%s\n', msg);
    input('Press ENTER to continue...', 's');
end