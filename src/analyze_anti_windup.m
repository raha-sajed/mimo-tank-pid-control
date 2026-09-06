function analyze_anti_windup()
%ANALYZE_ANTI_WINDUP Compare Lambda tuning with and without clamping anti-windup.

projectRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(fullfile(projectRoot, 'src')));

figuresDir = fullfile(projectRoot, 'figures');
if ~exist(figuresDir, 'dir'), mkdir(figuresDir); end

params = tank_parameters();
controller = compute_pid_gains("Lambda", params);

simWithoutAw = simulate_closed_loop(params, controller, antiWindup=false);
simWithAw = simulate_closed_loop(params, controller, antiWindup=true);
simWithoutAw.controller.name = 'Lambda without anti-windup';
simWithAw.controller.name = 'Lambda with anti-windup';

plot_temperature_response({simWithoutAw, simWithAw}, params, ...
    fullfile(figuresDir, 'lambda_anti_windup_temperature.png'));
plot_actuator_response({simWithoutAw, simWithAw}, params, ...
    fullfile(figuresDir, 'lambda_anti_windup_heater_power.png'));
end
