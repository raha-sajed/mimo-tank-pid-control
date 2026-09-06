function plot_actuator_zoom(simulations, params, outputPath, options)
%PLOT_ACTUATOR_ZOOM Plot zoomed heater-power response in kW.

arguments
    simulations
    params struct
    outputPath string = ""
    options.mode string = "setpoint"   % "setpoint" or "disturbance"
end

if ~iscell(simulations)
    simulations = {simulations};
end

figure('Name', 'Zoomed Heater Power Response', 'NumberTitle', 'off', 'Color', 'w');
hold on; grid on; box on;
set(gca, 'FontName', 'Arial', 'FontSize', 10, 'LineWidth', 1);

switch lower(options.mode)
    case "disturbance"
        xLimits = [2400 3200];
        titleText = 'Heater Power Detail During Inlet-Temperature Disturbance';
        eventTime = params.disturbance_time;
        eventLabel = 'Tin disturbance';
        regionColor = [1.00 0.94 0.86];
    otherwise
        xLimits = [450 900];
        titleText = 'Heater Power Detail During Setpoint Tracking';
        eventTime = params.setpoint_step_time;
        eventLabel = 'Setpoint step';
        regionColor = [0.88 0.94 1.00];
end

% Determine readable y-limits from the selected time interval.
allY = [];
for i = 1:numel(simulations)
    sim = simulations{i};
    idx = sim.t >= xLimits(1) & sim.t <= xLimits(2);
    allY = [allY; sim.Qh(idx) / 1000]; %#ok<AGROW>
end
allY = allY(isfinite(allY));
yMin = max(params.Qh_min / 1000, min(allY) - 0.08 * range(allY));
yMax = min(params.Qh_max / 1000, max(allY) + 0.08 * range(allY));
if yMin == yMax
    yMin = yMin - 1;
    yMax = yMax + 1;
end

patch([xLimits(1) xLimits(2) xLimits(2) xLimits(1)], ...
      [yMin yMin yMax yMax], regionColor, ...
      'EdgeColor', 'none', 'FaceAlpha', 0.35, 'HandleVisibility', 'off');

colors = lines(numel(simulations));
for i = 1:numel(simulations)
    sim = simulations{i};
    qhKW = sim.Qh / 1000;
    plot(sim.t, qhKW, 'LineWidth', 1.6, 'Color', colors(i,:), ...
        'DisplayName', sim.controller.name);

    % Marker at local maximum for setpoint plot, local final/peak deviation for disturbance plot.
    idx = sim.t >= xLimits(1) & sim.t <= xLimits(2);
    tRegion = sim.t(idx);
    qRegion = qhKW(idx);
    if lower(options.mode) == "disturbance"
        [~, k] = max(abs(qRegion - params.Qh0/1000));
    else
        [~, k] = max(qRegion);
    end
    plot(tRegion(k), qRegion(k), 'o', 'Color', colors(i,:), ...
        'MarkerFaceColor', colors(i,:), 'MarkerSize', 4, ...
        'HandleVisibility', 'off');
end

xline(eventTime, '--k', eventLabel, 'LabelOrientation', 'horizontal', ...
    'LabelVerticalAlignment', 'bottom', 'HandleVisibility', 'off');
yline(params.Qh0 / 1000, 'k--', 'Nominal heater power', ...
    'HandleVisibility', 'off');

xlabel('Time (s)');
ylabel('Heater Power (kW)');
title(titleText, 'FontWeight', 'bold');
xlim(xLimits); ylim([yMin yMax]);
legend('Location', 'best', 'FontSize', 8);

if strlength(outputPath) > 0
    exportgraphics(gcf, outputPath, 'Resolution', 300);
end
end
