function plot_linear_speed_comparison_matlab(speed50Csv, speed100Csv, outputDir)
%PLOT_LINEAR_SPEED_COMPARISON_MATLAB Compare expected and actual triangles.

if nargin < 1
    speed50Csv = fullfile( ...
        'artifacts', 'cartesian_tracking', 'left_linear50_20260902.csv');
end
if nargin < 2
    speed100Csv = fullfile( ...
        'artifacts', 'cartesian_tracking', ...
        'left_linear100_circular50_20260902.csv');
end
if nargin < 3
    outputDir = fullfile('artifacts', 'cartesian_tracking', 'matlab');
end
if ~exist(outputDir, 'dir')
    mkdir(outputDir);
end

speed50 = pathRows(readtable(speed50Csv, 'VariableNamingRule', 'preserve'));
speed100 = pathRows(readtable(speed100Csv, 'VariableNamingRule', 'preserve'));

sideLength = 0.14;
centerY = 0.231892;
centerZ = 0.381638;
radius = sideLength / sqrt(3);
expectedY = [ ...
    centerY + radius, ...
    centerY - radius / 2, ...
    centerY - radius / 2, ...
    centerY + radius];
expectedZ = [ ...
    centerZ, centerZ + sideLength / 2, ...
    centerZ - sideLength / 2, centerZ];

figure('Color', 'w', 'Position', [100, 100, 1450, 680]);
layout = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
drawComparison(speed50, expectedY, expectedZ, ...
    'Linear speed 50: expected vs actual');
drawComparison(speed100, expectedY, expectedZ, ...
    'Linear speed 100: expected vs actual');
title(layout, 'Left-arm 14 cm triangle tracking');
exportgraphics(gcf, ...
    fullfile(outputDir, 'linear_speed_50_vs_100.png'), ...
    'Resolution', 180);

fprintf('Speed 50 RMS / P95 / Max (mm): %.3f / %.3f / %.3f\n', ...
    pathRms(speed50), pathPercentile95(speed50), pathMaximum(speed50));
fprintf('Speed 100 RMS / P95 / Max (mm): %.3f / %.3f / %.3f\n', ...
    pathRms(speed100), pathPercentile95(speed100), pathMaximum(speed100));
end

function rows = pathRows(tableValue)
rows = tableValue( ...
    strcmp(tableValue.motion, 'linear_triangle') & ...
    strcmp(tableValue.phase, 'path'), :);
end

function drawComparison(rows, expectedY, expectedZ, titleText)
nexttile;
plot(100 * expectedY, 100 * expectedZ, 'r--', 'LineWidth', 2.0);
hold on;
plot(100 * rows.y_m, 100 * rows.z_m, 'b-', 'LineWidth', 1.5);
axis equal;
grid on;
xlabel('Y (cm)');
ylabel('Z (cm)');
title(titleText);
legend('Expected geometry', 'Actual TCP feedback', 'Location', 'best');
end

function value = pathRms(rows)
errorMm = 1000 * rows.path_error_m;
value = sqrt(mean(errorMm.^2));
end

function value = pathMaximum(rows)
value = max(1000 * rows.path_error_m);
end

function value = pathPercentile95(rows)
values = sort(1000 * rows.path_error_m);
position = 1 + 0.95 * (numel(values) - 1);
lowerIndex = floor(position);
upperIndex = ceil(position);
if lowerIndex == upperIndex
    value = values(lowerIndex);
else
    weight = position - lowerIndex;
    value = values(lowerIndex) * (1 - weight) + values(upperIndex) * weight;
end
end
