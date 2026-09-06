function resultsTable = compare_pid_methods()
%COMPARE_PID_METHODS Compare classical PID tuning methods on the nonlinear tank.

projectRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(fullfile(projectRoot, 'src')));

figuresDir = fullfile(projectRoot, 'figures');
resultsDir = fullfile(projectRoot, 'results');
if ~exist(figuresDir, 'dir'), mkdir(figuresDir); end
if ~exist(resultsDir, 'dir'), mkdir(resultsDir); end

params = tank_parameters();
methods = ["Ziegler-Nichols", "CHR", "Cohen-Coon", "Lambda", ...
           "Maximally Flat", "Haalman", "AMIGO"];

simulations = cell(numel(methods), 1);
metrics = cell(numel(methods), 1);

for i = 1:numel(methods)
    controller = compute_pid_gains(methods(i), params);
    simulations{i} = simulate_closed_loop(params, controller);
    metrics{i} = compute_performance_metrics(simulations{i}, params);
end

resultsTable = struct2table([metrics{:}]');
writetable(resultsTable, fullfile(resultsDir, 'controller_comparison_metrics.csv'));

plot_temperature_response(simulations, params, fullfile(figuresDir, 'controller_temperature_comparison.png'), mode="full");
plot_temperature_response(simulations, params, fullfile(figuresDir, 'setpoint_tracking_zoom.png'), mode="setpoint", showMarkers=true);
plot_temperature_response(simulations, params, fullfile(figuresDir, 'disturbance_rejection_zoom.png'), mode="disturbance", showMarkers=true);
plot_actuator_response(simulations, params, fullfile(figuresDir, 'heater_power_comparison_kw.png'));
plot_actuator_zoom(simulations, params, fullfile(figuresDir, 'heater_power_setpoint_zoom.png'), mode="setpoint");
plot_actuator_zoom(simulations, params, fullfile(figuresDir, 'heater_power_disturbance_zoom.png'), mode="disturbance");

disp(resultsTable);
fprintf('\nSaved results to: %s\n', resultsDir);
fprintf('Saved figures to: %s\n', figuresDir);
end
