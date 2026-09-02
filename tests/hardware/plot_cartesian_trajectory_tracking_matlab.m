function plot_cartesian_trajectory_tracking_matlab(csvPath, outputDir)
%PLOT_CARTESIAN_TRAJECTORY_TRACKING_MATLAB Compare declared and measured paths.

if nargin < 1
    csvPath = fullfile( ...
        'artifacts', 'cartesian_tracking', ...
        'left_linear100_circular50_20260902.csv');
end
if nargin < 2
    outputDir = fullfile('artifacts', 'cartesian_tracking', 'matlab');
end
if ~exist(outputDir, 'dir')
    mkdir(outputDir);
end

T = readtable(csvPath, 'VariableNamingRule', 'preserve');
triangle = T(strcmp(T.motion, 'linear_triangle'), :);
circle = T(startsWith(T.motion, 'circular_'), :);
circleOutward = T(strcmp(T.motion, 'circular_outward'), :);
circleReturn = T(strcmp(T.motion, 'circular_return'), :);
trianglePath = triangle(strcmp(triangle.phase, 'path'), :);
circlePath = circle(strcmp(circle.phase, 'path'), :);
circleOutwardPath = circleOutward(strcmp(circleOutward.phase, 'path'), :);
circleReturnPath = circleReturn(strcmp(circleReturn.phase, 'path'), :);

sideLength = 0.14;
centerY = 0.231892;
centerZ = 0.381638;
radius = sideLength / sqrt(3);
triangleExpectedY = [ ...
    centerY + radius, ...
    centerY - radius / 2, ...
    centerY - radius / 2, ...
    centerY + radius];
triangleExpectedZ = [ ...
    centerZ, centerZ + sideLength / 2, ...
    centerZ - sideLength / 2, centerZ];

theta = linspace(-pi / 2, 3 * pi / 2, 721);
circleExpectedY = centerY + 0.10 * cos(theta);
circleExpectedZ = centerZ + 0.10 * sin(theta);

figure('Color', 'w', 'Position', [100, 100, 1450, 650]);
layout = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile;
plot(100 * triangleExpectedY, 100 * triangleExpectedZ, ...
    'r--', 'LineWidth', 2.0);
hold on;
plot(100 * trianglePath.y_m, 100 * trianglePath.z_m, ...
    'b-', 'LineWidth', 1.5);
axis equal;
grid on;
xlabel('Y (cm)');
ylabel('Z (cm)');
title('Linear triangle, speed 100');
legend('Expected geometry', 'Actual TCP feedback', 'Location', 'best');

nexttile;
plot(100 * circleExpectedY, 100 * circleExpectedZ, ...
    'r--', 'LineWidth', 2.0);
hold on;
plot(100 * circlePath.y_m, 100 * circlePath.z_m, ...
    'b-', 'LineWidth', 1.5);
axis equal;
grid on;
xlabel('Y (cm)');
ylabel('Z (cm)');
title('Circular full circle, speed 50');
legend('Expected geometry', 'Actual TCP feedback', 'Location', 'best');

title(layout, 'Declared Cartesian path vs measured TCP path');
exportgraphics(gcf, ...
    fullfile(outputDir, 'cartesian_path_expected_vs_actual.png'), ...
    'Resolution', 180);

figure('Color', 'w', 'Position', [100, 100, 1450, 650]);
layout = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
nexttile;
plot(trianglePath.elapsed_s, 1000 * trianglePath.path_error_m, ...
    'b-', 'LineWidth', 1.4);
grid on;
xlabel('Time since submission (s)');
ylabel('Nearest-path error (mm)');
title('Linear triangle tracking error');
nexttile;
plot(circleOutwardPath.elapsed_s, 1000 * circleOutwardPath.path_error_m, ...
    'b-', 'LineWidth', 1.4);
hold on;
returnTime = circleReturnPath.elapsed_s + circleOutwardPath.elapsed_s(end);
plot(returnTime, 1000 * circleReturnPath.path_error_m, ...
    'm-', 'LineWidth', 1.4);
grid on;
xlabel('Combined path time (s)');
ylabel('Nearest-path error (mm)');
title('Circular tracking error');
legend('Outward half', 'Return half', 'Location', 'best');
title(layout, 'Cartesian geometric tracking error');
exportgraphics(gcf, ...
    fullfile(outputDir, 'cartesian_tracking_error.png'), ...
    'Resolution', 180);

fprintf('Linear path RMS / P95 / Max (mm): %.3f / %.3f / %.3f\n', ...
    sqrt(mean((1000 * trianglePath.path_error_m).^2)), ...
    percentile95(1000 * trianglePath.path_error_m), ...
    max(1000 * trianglePath.path_error_m));
fprintf('Circular path RMS / P95 / Max (mm): %.3f / %.3f / %.3f\n', ...
    sqrt(mean((1000 * circlePath.path_error_m).^2)), ...
    percentile95(1000 * circlePath.path_error_m), ...
    max(1000 * circlePath.path_error_m));
end

function value = percentile95(values)
values = sort(values(:));
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
