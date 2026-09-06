function params = tank_parameters()
%TANK_PARAMETERS Return nominal parameters for the MIMO stirred-tank system.
%
% Outputs:
%   params - structure containing physical constants, operating point,
%            actuator limits, and simulation scenario settings.

params.A_tank = 0.785;          % Tank cross-sectional area [m^2]
params.kv = 0.001;              % Outlet valve coefficient
params.rho = 1000;              % Water density [kg/m^3]
params.Cp = 4180;               % Specific heat capacity [J/(kg.K)]
params.inv_rho_Cp = 1 / (params.rho * params.Cp);

% Nominal operating point
params.h0 = 1.0;                % Operating liquid level [m]
params.T0 = 50.0;               % Operating tank temperature [degC]
params.T_initial = 45.0;        % Initial temperature for setpoint tests [degC]
params.Tin_nominal = 25.0;      % Nominal inlet temperature [degC]
params.Tin_disturbed = 20.0;    % Disturbed inlet temperature [degC]
params.qin0 = 0.001;            % Nominal inlet flow rate [m^3/s]
params.Qh0 = 104500;            % Nominal heater power [W]

% FOPTD approximation used for PID tuning
params.K = 0.0002392;           % Static process gain for temperature loop
params.Tau = 785;               % Process time constant [s]
params.L = 20;                  % Effective time delay [s]

% Baseline level controller and feedforward decoupler
params.Kp_level = 0.013083;
params.Ki_level = 0.000008;
params.D21 = 1.045e8;

% Actuator limits
params.qin_min = 0;
params.qin_max = 0.01;
params.Qh_min = 0;
params.Qh_max = 500000;

% Simulation settings
params.tspan = [0 5000];
params.setpoint_step_time = 500;
params.disturbance_time = 2500;
params.T_setpoint_before_step = 45;
params.T_setpoint_after_step = 50;
params.initial_state = [params.h0; params.T_initial; 0; 0];
end
