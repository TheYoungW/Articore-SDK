function plot_ordinary_pv_target_matlab(csvPath, outputDir)
%PLOT_ORDINARY_PV_TARGET_MATLAB Plot all 14 ordinary-PV feedback joints.
%   Uses the target, actual position, velocity, error and frame-delta columns
%   emitted by measure_ordinary_pv_target.py. Runtime's internal per-cycle
%   P/V reference is not exposed and is not reconstructed here.

if nargin < 1
    csvPath = fullfile( ...
        'artifacts', 'ordinary_pv_target', ...
        'ordinary_pv_target.csv');
end
if nargin < 2
    outputDir = fullfile('artifacts', 'ordinary_pv_target', 'matlab');
end
if ~exist(outputDir, 'dir')
    mkdir(outputDir);
end

T = readtable(csvPath, 'VariableNamingRule', 'preserve');
if isempty(T)
    error('The PV measurement CSV contains no samples: %s', csvPath);
end
t = T.elapsed_s;
jointNames = [
    "left/J1", "left/J2", "left/J3", "left/J4", "left/J5", "left/J6", "left/J7", ...
    "right/J1", "right/J2", "right/J3", "right/J4", "right/J5", "right/J6", "right/J7"
];
positionDeg = zeros(height(T), numel(jointNames));
velocityDegS = zeros(height(T), numel(jointNames));
targetDeg = zeros(1, numel(jointNames));
errorDeg = zeros(height(T), numel(jointNames));
frameDeltaDeg = zeros(height(T), numel(jointNames));
for index = 1:numel(jointNames)
    positionDeg(:, index) = T.(jointNames(index) + "_position_deg");
    velocityDegS(:, index) = T.(jointNames(index) + "_velocity_deg_s");
    targetColumn = T.(jointNames(index) + "_target_deg");
    targetDeg(index) = targetColumn(1);
    errorDeg(:, index) = T.(jointNames(index) + "_error_deg");
    frameDeltaDeg(:, index) = T.(jointNames(index) + "_frame_delta_deg");
end

% Fill each tiled row as left/Jn, right/Jn.
shown = reshape([1:7; 8:14], 1, []);
holdMask = string(T.phase) == "hold";
holdStart = NaN;
if any(holdMask)
    holdStart = t(find(holdMask, 1, 'first'));
end

figure('Color', 'w', 'Position', [80, 40, 1500, 2100]);
layout = tiledlayout(7, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
for tile = 1:numel(shown)
    index = shown(tile);
    nexttile;
    expected = repmat(targetDeg(index), size(t));
    plot(t, expected, 'r--', 'LineWidth', 1.5);
    hold on;
    plot(t, positionDeg(:, index), 'b-', 'LineWidth', 1.4);
    if ~isnan(holdStart)
        xline(holdStart, ':', 'Hold', 'Color', [0.2, 0.55, 0.2]);
    end
    grid on;
    xlabel('Time (s)');
    ylabel('Position (deg)');
    title(strrep(jointNames(index), '/', ' '));
    legend('Expected command', 'Actual feedback', 'Location', 'best');
end
title(layout, 'Ordinary PV: all 14 joints, target vs actual feedback');
exportgraphics(gcf, fullfile(outputDir, 'ordinary_pv_position_comparison.png'), ...
    'Resolution', 180);

figure('Color', 'w', 'Position', [80, 40, 1500, 2100]);
layout = tiledlayout(7, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
for tile = 1:numel(shown)
    index = shown(tile);
    nexttile;
    plot(t, errorDeg(:, index), 'b-', 'LineWidth', 1.2);
    hold on;
    yline(0, 'k--');
    if ~isnan(holdStart)
        xline(holdStart, ':', 'Hold', 'Color', [0.2, 0.55, 0.2]);
    end
    grid on;
    xlabel('Time (s)');
    ylabel('Error (deg)');
    title(strrep(jointNames(index), '/', ' '));
end
title(layout, 'Ordinary PV: target error of all 14 joints');
exportgraphics(gcf, fullfile(outputDir, 'ordinary_pv_position_error.png'), ...
    'Resolution', 180);

if any(holdMask)
    holdTime = t(holdMask);
    figure('Color', 'w', 'Position', [80, 40, 1500, 2100]);
    layout = tiledlayout(7, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    for tile = 1:numel(shown)
        index = shown(tile);
        holdPosition = positionDeg(holdMask, index);
        holdJitter = holdPosition - mean(holdPosition);
        nexttile;
        plot(holdTime, holdJitter, 'b-', 'LineWidth', 1.2);
        hold on;
        yline(0, 'k--');
        grid on;
        xlabel('Time (s)');
        ylabel('Position - hold mean (deg)');
        title(strrep(jointNames(index), '/', ' '));
    end
    title(layout, 'Ordinary PV: steady-state jitter around each joint hold mean');
    exportgraphics(gcf, fullfile(outputDir, 'ordinary_pv_hold_jitter.png'), ...
        'Resolution', 180);
else
    warning(['No hold-phase samples were recorded. The hold-jitter figure ', ...
             'cannot be generated; increase --timeout or relax settling thresholds.']);
end

figure('Color', 'w', 'Position', [80, 40, 1500, 2100]);
layout = tiledlayout(7, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
for tile = 1:numel(shown)
    index = shown(tile);
    nexttile;
    plot(t, frameDeltaDeg(:, index), 'b-', 'LineWidth', 1.0);
    hold on;
    yline(0, 'k--');
    if ~isnan(holdStart)
        xline(holdStart, ':', 'Hold', 'Color', [0.2, 0.55, 0.2]);
    end
    grid on;
    xlabel('Time (s)');
    ylabel('\Delta position / feedback frame (deg)');
    title(strrep(jointNames(index), '/', ' '));
end
title(layout, 'Ordinary PV: adjacent-feedback position change');
exportgraphics(gcf, fullfile(outputDir, 'ordinary_pv_frame_delta.png'), ...
    'Resolution', 180);

fprintf(['Joint       Peak velocity   Final error   Hold p-p   Hold std   ', ...
         'Hold max frame step\n']);
for index = 1:numel(jointNames)
    if any(holdMask)
        holdPosition = positionDeg(holdMask, index);
        holdFrameDelta = frameDeltaDeg(holdMask, index);
        holdPeakToPeak = max(holdPosition) - min(holdPosition);
        holdStd = std(holdPosition, 1);
        holdMaxFrameStep = max(abs(holdFrameDelta));
    else
        holdPeakToPeak = NaN;
        holdStd = NaN;
        holdMaxFrameStep = NaN;
    end
    fprintf('%-10s  %10.3f      %10.4f   %9.5f  %9.5f  %12.6f\n', ...
        jointNames(index), max(abs(velocityDegS(:, index))), ...
        errorDeg(end, index), holdPeakToPeak, holdStd, holdMaxFrameStep);
end
fprintf('MATLAB figures written to: %s\n', outputDir);
drawnow;
end
