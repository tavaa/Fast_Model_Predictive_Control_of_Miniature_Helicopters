% SCRIPT: CONVERGENCE ANALYSIS FOR LTV-MPC
% Calculates and plots the tracking error over time for the 3
% initial conditions.

clear; clc; close all;

%% SETUP DIR
[scriptDir, ~, ~] = fileparts(mfilename('fullpath'));
resultsBaseDir = fullfile(scriptDir, '../results/MPC/PWC_reference/');

% folder to save figures
figSaveDir = fullfile(scriptDir, '../plots/matlab/PWC_reference/ConvergencePlots_MPC/');
if ~exist(figSaveDir, 'dir')
    mkdir(figSaveDir);
end

% Specify scenario
targetFolders = {'Ts002', 'Ts005', 'Ts010', 'Ts020'};
targetFoldersTitle = {'0.02 Ts', '0.05 Ts', '0.10 Ts', '0.20 Ts'};
scenario_target = 'SpiralEllipse';

%% ITERATE OVER TS
for i = 1:length(targetFolders)

    runDir = fullfile(resultsBaseDir, targetFolders{i});

    % check exists
    if ~exist(runDir, 'dir')
        error('[ERROR] Cannot find: %s', runDir);
    end

    fprintf('[ANALYSIS] Loading data from: %s\n', targetFolders{i});

    % load conditions
    conditions = {'OnReference', 'Perturbation', 'Origin'};
    colors = {'#77AC30', '#0072BD', '#D95319'}; 
    line_styles = {'-.', '-.', '-.'};

    convergence_thresholds =[0.05, 0.10, 0.20, 0.50]; 
    
    fig = figure('Color', 'w', 'Position',[200, 200, 1000, 500]);
    hold on; grid on;

    xlabel('Time [s]', 'FontSize', 12, 'FontWeight', 'bold');
    ylabel('Position Error [m]', 'FontSize', 12, 'FontWeight', 'bold');
    title(sprintf('Convergence Analysis: %s, %s', scenario_target, targetFoldersTitle{i}), 'FontSize', 14);

    if strcmp(scenario_target, "SpiralEllipse")
        ylim([0, 3.5]);
    else
        ylim([0,1]);
    end

    % Draw thresholds
    h_thresh = zeros(1,length(convergence_thresholds));
    for j = 1:length(convergence_thresholds)
        h_thresh(j) = yline(convergence_thresholds(j), 'k--', 'LineWidth', 0.5);
    end

    h_plots = zeros(1, length(conditions));
    legend_entries = cell(1, length(conditions));

    %% PLOT DATA from csv
    for c = 1:length(conditions)
        cond_name = conditions{c};
        filename = fullfile(runDir, sprintf('MPC_Results_%s_%s.csv', scenario_target, cond_name));

        if ~exist(filename, 'file')
            warning('File not found: %s', filename);
            continue;
        end

        % read from csv
        data = readtable(filename);

        % Norm of the error
        err = sqrt((data.X - data.Ref_X).^2 + ...
                (data.Y - data.Ref_Y).^2 + ...
                (data.Z - data.Ref_Z).^2);
                %(data.Psi - data.Ref_Psi).^2);

        % Plot error
        h_plots(c) = plot(data.Time, err, ...
            'Color', colors{c}, ...
            'LineStyle', line_styles{c}, ...
            'LineWidth', 2.5);

        legend_entries{c} = sprintf('%s', cond_name);
    end

    %% ADD LEGEND
    legend([h_plots h_thresh], ...
        [legend_entries, ...
        arrayfun(@(x) sprintf('Threshold %.2f m', x), ...
        convergence_thresholds, 'UniformOutput', false)], ...
        'Location', 'northeast');

    %% SAVE FIGURE AS PNG
    saveName = sprintf('Convergence_%s_%s.png', ...
        scenario_target, targetFolders{i});
    
    exportgraphics(fig, fullfile(figSaveDir, saveName), ...
        'Resolution', 300);

    fprintf('[SAVED] %s\n', saveName);

end