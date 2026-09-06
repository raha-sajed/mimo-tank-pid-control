function plot_temperature_response(simulations, params, outputPath, options)
%PLOT_TEMPERATURE_RESPONSE Plot publication-quality temperature responses.

arguments
    simulations
    params struct
    outputPath string = ""
    options.mode string = "full"   % "full", "setpoint", or "disturbance"
    options.showMarkers logical = false
end

if ~iscell(simulations)
    simulations = {simulations};
end

figure('Name', 'MIMO Tank Temperature Response', 'NumberTitle', 'off', 'Color', 'w');
hold on; grid on; box on;
set(gca, 'FontName', 'Arial', 'FontSize', 10, 'LineWidth', 1);

switch lower(options.mode)
    case "setpoint"
        xLimits = [400 1200];
        yLimits = [49.2 52.0];
        titleText = 'Setpoint Tracking Detail';
    case "disturbance"
        xLimits = [2400 3400];
        yLimits = [49.65 50.10];
        titleText = 'Disturbance Rejection Detail';
    otherwise
        xLimits = params.tspan;
        yLimits = [44 52];
        titleText = 'Controller Comparison: Setpoint Tracking and Disturbance Rejection';
end

add_background_regions(params, yLimits);
colors = lines(numel(simulations));

for i = 1:numel(simulations)
    sim = simulations{i};
    plot(sim.t, sim.T, 'LineWidth', 1.8, 'Color', colors(i, :), ...
        'DisplayName', sim.controller.name);

    if options.showMarkers
        add_key_markers(sim, params, colors(i, :), options.mode);
    end
end

plot_reference(params);
add_event_lines(params);
add_region_labels(params, yLimits, options.mode);

xlabel('Time (s)');
ylabel('Temperature (°C)');
title(titleText, 'FontWeight', 'bold');
xlim(xLimits); ylim(yLimits);
legend('Location', 'best', 'FontSize', 8);

if strlength(outputPath) > 0
    exportgraphics(gcf, outputPath, 'Resolution', 300);
end
end

function add_background_regions(params, yLimits)
regions = [params.tspan(1), params.setpoint_step_time; ...
           params.setpoint_step_time, params.disturbance_time; ...
           params.disturbance_time, params.tspan(2)];
colors = [0.94 0.94 0.94; 0.88 0.94 1.00; 1.00 0.94 0.86];
for r = 1:3
    patch([regions(r,1) regions(r,2) regions(r,2) regions(r,1)], ...
          [yLimits(1) yLimits(1) yLimits(2) yLimits(2)], colors(r,:), ...
          'EdgeColor', 'none', 'FaceAlpha', 0.35, 'HandleVisibility', 'off');
end
end

function plot_reference(params)
tRef = [params.tspan(1), params.setpoint_step_time, params.setpoint_step_time, params.tspan(2)];
yRef = [params.T_setpoint_before_step, params.T_setpoint_before_step, ...
        params.T_setpoint_after_step, params.T_setpoint_after_step];
plot(tRef, yRef, 'k--', 'LineWidth', 1.4, 'DisplayName', 'Reference');
end

function add_event_lines(params)
xline(params.setpoint_step_time, '--k', 'Setpoint step', ...
    'LabelOrientation', 'horizontal', 'LabelVerticalAlignment', 'bottom', ...
    'HandleVisibility', 'off');
xline(params.disturbance_time, '--k', 'Tin disturbance', ...
    'LabelOrientation', 'horizontal', 'LabelVerticalAlignment', 'bottom', ...
    'HandleVisibility', 'off');
end

function add_region_labels(params, yLimits, mode)
if lower(mode) ~= "full"
    return;
end
yText = yLimits(2) - 0.35;
text(mean([params.tspan(1), params.setpoint_step_time]), yText, 'Initial condition', ...
    'HorizontalAlignment', 'center', 'FontSize', 9);
text(mean([params.setpoint_step_time, params.disturbance_time]), yText, 'Setpoint tracking', ...
    'HorizontalAlignment', 'center', 'FontSize', 9);
text(mean([params.disturbance_time, params.tspan(2)]), yText, 'Disturbance rejection', ...
    'HorizontalAlignment', 'center', 'FontSize', 9);
end

function add_key_markers(sim, params, color, mode)
if lower(mode) == "setpoint"
    idx = sim.t >= params.setpoint_step_time & sim.t <= params.disturbance_time;
    tRegion = sim.t(idx); TRegion = sim.T(idx);
    [Tpeak, k] = max(TRegion);
    plot(tRegion(k), Tpeak, 'o', 'Color', color, 'MarkerFaceColor', color, ...
        'MarkerSize', 5, 'HandleVisibility', 'off');
elseif lower(mode) == "disturbance"
    idx = sim.t >= params.disturbance_time;
    tRegion = sim.t(idx); TRegion = sim.T(idx);
    [Tmin, k] = min(TRegion);
    plot(tRegion(k), Tmin, 'v', 'Color', color, 'MarkerFaceColor', color, ...
        'MarkerSize', 5, 'HandleVisibility', 'off');
end
end
