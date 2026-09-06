clear; clc; close all;

%% Step 1: System Constants & Tuning Parameters
global A_tank kv Tin_nominal inv_rho_Cp Kp_level Ki_level Kp_temp Ki_temp D21 h0 T0
A_tank = 0.785;       
kv = 0.001;           
Tin_nominal = 25;     
inv_rho_Cp = 2.392e-7;
h0 = 1; T0 = 50;      

% Controller & Decoupler Gains (Calculated previously)
Kp_level = 0.013083;  Ki_level = 0.000008;
Kp_temp = 109392.419; Ki_temp = 139.353;
D21 = 1.045e08;

%% Step 2: Nonlinear Simulation Configuration
tspan = [0 5000]; % Simulation time: 5000 seconds
% Initial conditions in absolute values: h(0) = 1m, T(0) = 50C
% We also initialize the integrator states for PI controllers: error_int_h=0, error_int_T=0
x0 = [1; 50; 0; 0]; 

% Run Nonlinear Solver
[t, x] = ode45(@tank_nonlinear_system, tspan, x0);

%% Step 3: Plotting the Results
figure('Name', 'Nonlinear Pure MATLAB Simulation', 'NumberTitle', 'off');

subplot(2,1,1);
plot(t, x(:,1), 'b', 'LineWidth', 2);
grid on;
title('Nonlinear Tank Level Response (h)');
xlabel('Time (sec)'); ylabel('Height (m)');

subplot(2,1,2);
plot(t, x(:,2), 'r', 'LineWidth', 2);
grid on;
title('Nonlinear Tank Temperature Response to Tin Disturbance (T)');
xlabel('Time (sec)'); ylabel('Temperature (C)');

%% Step 4: Nonlinear System Dynamics with Controllers & Disturbance
function dxdt = tank_nonlinear_system(t, x)
    global A_tank kv Tin_nominal inv_rho_Cp Kp_level Ki_level Kp_temp Ki_temp D21 h0 T0
    
    % Extract States
    h = x(1);           % Actual Tank Height
    T = x(2);           % Actual Tank Temperature
    int_err_h = x(3);   % Level Controller Integrator State
    int_err_T = x(4);   % Temp Controller Integrator State
    
    % Define Disturbance: At t = 500s, Tin drops by 5 degrees (from 25 to 20)
    if t >= 500
        Tin = 20; 
    else
        Tin = Tin_nominal;
    end
    
    % Calculate Errors (Deviation from Operating Points)
    % Let's assume setpoints are at operating points (Delta_h_sp = 0, Delta_T_sp = 0)
    err_h = h0 - h; 
    err_T = T0 - T;
    
    % PI Controllers Output (Deviation variables)
    delta_u1 = Kp_level * err_h + Ki_level * int_err_h;
    delta_u2_fb = Kp_temp * err_T + Ki_temp * int_err_T;
    
    % Feedforward Decoupler Action
    delta_u2_ff = D21 * delta_u1;
    
    % Total Control Inputs (Absolute Values = Operating Point + Deviation)
    qin0 = 0.001;
    Qh0 = 104500;
    
    qin = qin0 + delta_u1;
    Qh = Qh0 + delta_u2_fb + delta_u2_ff;
    
    % Saturation limits for physical actuators (Safety & Reality Check)
    qin = max(0, qin); 
    Qh = max(0, Qh);
    
    % Nonlinear Differential Equations
    dh_dt = (1 / A_tank) * (qin - kv * sqrt(h));
    dT_dt = (qin / (A_tank * h)) * (Tin - T) + (Qh * inv_rho_Cp / (A_tank * h));
    
    % Derivative of Integrator States
    d_int_err_h = err_h;
    d_int_err_T = err_T;
    
    % Return Derivatives
    dxdt = [dh_dt; dT_dt; d_int_err_h; d_int_err_T];
end