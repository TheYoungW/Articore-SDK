function plot_ordinary_pv_target_matlab(csvPath, outputDir)
%PLOT_ORDINARY_PV_TARGET_MATLAB Compare ordinary-PV targets with feedback.
%   The target curves are the final joint endpoints submitted by
%   set_joint_pv(). Runtime's internal per-cycle P/V reference is not exposed
%   by the SDK and therefore is not reconstructed here.

if nargin < 1
    csvPath = fullfile( ...
        'artifacts', 'ordinary_pv_target', ...
        'ordinary_pv_left_j1_minus30_speed30_20260902.csv');
end
if nargin < 2
    outputDir = fullfile('artifacts', 'ordinary_pv_target', 'matlab');
end
if ~exist(outputDir, 'dir')
    mkdir(outputDir);
end

T = readtable(csvPath, 'VariableNamingRule', 'preserve');
t = T.elapsed_s;
jointNames = [
    "left/J1", "left/J2", "left/J3", "left/J4", "left/J5", "left/J6", "left/J7", ...
    "right/J1", "right/J2", "right/J3", "right/J4", "right/J5", "right/J6", "right/J7"
];
targetDeg = [-30, 0, 0, 90, 0, 0, 0, 0, 0, 0, 90, 0, 0, 0];
positionDeg = zeros(height(T), numel(jointNames));
velocityDegS = zeros(height(T), numel(jointNames));
for index = 1:numel(jointNames)
    positionDeg(:, index) = rad2deg(T.(jointNames(index) + "_position_rad"));
    velocityDegS(:, index) = rad2deg(T.(jointNames(index) + "_velocity_rad_s"));
end
errorDeg = positionDeg - targetDeg;

shown = 1:14;
figure('Color', 'w', 'Position', [80, 40, 1500, 2100]);
layout = tiledlayout(7, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
for tile = 1:numel(shown)
    index = shown(tile);
    nexttile;
    expected = repmat(targetDeg(index), size(t));
    plot(t, expected, 'r--', 'LineWidth', 1.5);
    hold on;
    plot(t, positionDeg(:, index), 'b-', 'LineWidth', 1.4);
    grid on;
    xlabel('Time (s)');
    ylabel('Position (deg)');
    title(strrep(jointNames(index), '/', ' '));
    legend('Expected command', 'Actual feedback', 'Location', 'best');
end
title(layout, 'Ordinary PV: all 14 joints, expected vs actual (speed 30)');
exportgraphics(gcf, fullfile(outputDir, 'ordinary_pv_position_comparison.png'), ...
    'Resolution', 180);

fprintf('Joint       Peak velocity (deg/s)   Final error (deg)\n');
for index = shown
    fprintf('%-10s  %10.3f               %10.4f\n', ...
        jointNames(index), max(abs(velocityDegS(:, index))), errorDeg(end, index));
end
end
