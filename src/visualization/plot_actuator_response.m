function plot_actuator_response(simulations, params, outputPath)
%PLOT_ACTUATOR_RESPONSE Plot heater power in kW with event annotations.

if ~iscell(simulations)
    simulations = {simulations};
end

figure('Name', 'Heater Power Response', 'NumberTitle', 'off', 'Color', 'w');
hold on; grid on; box on;
set(gca, 'FontName', 'Arial', 'FontSize', 10, 'LineWidth', 1);

colors = lines(numel(simulations));
for i = 1:numel(simulations)
    sim = simulations{i};
    plot(sim.t, sim.Qh / 1000, 'LineWidth', 1.5, 'Color', colors(i,:), ...
        'DisplayName', sim.controller.name);
end

yline(params.Qh_min / 1000, 'k:', 'Lower saturation', 'HandleVisibility', 'off');
yline(params.Qh_max / 1000, 'k:', 'Upper saturation', 'HandleVisibility', 'off');
xline(params.setpoint_step_time, '--k', 'Setpoint step', ...
    'LabelOrientation', 'horizontal', 'HandleVisibility', 'off');
xline(params.disturbance_time, '--k', 'Tin disturbance', ...
    'LabelOrientation', 'horizontal', 'HandleVisibility', 'off');

xlabel('Time (s)');
ylabel('Heater Power (kW)');
title('Heater Power Control Action', 'FontWeight', 'bold');
legend('Location', 'best', 'FontSize', 8);

if nargin >= 3 && strlength(string(outputPath)) > 0
    exportgraphics(gcf, outputPath, 'Resolution', 300);
end
end
