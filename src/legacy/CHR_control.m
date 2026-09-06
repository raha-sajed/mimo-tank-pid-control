
clear; clc; close all;

%% PART 1: System Constants & Baseline Setup
global A_tank kv Tin_nominal inv_rho_Cp h0 T0
global Kp_level Ki_level Kp_temp Ki_temp D21 current_mode
global Kp_c Ki_c Kd_c

A_tank = 0.785;       
kv = 0.001;           
Tin_nominal = 25;     
inv_rho_Cp = 2.392e-7;
h0 = 1; T0 = 50; % Target final operating point

% Decentralized and decoupling loops
Kp_level = 0.013083;  Ki_level = 0.000008;
Kp_temp = 109392.419; Ki_temp = 139.353;
D21 = 1.045e08;

%% PART 2: CHR Parameter Calculations (Four Distinct Modes)
K = 0.0002392; Tau = 785; L = 20;
base_gain = Tau / (K * L);

% Mode 1: Set-point Tracking with 0% Overshoot
Kp_SP_0 = 0.6 * base_gain;   Ti_SP_0 = Tau;        Td_SP_0 = 0.5 * L;
Ki_SP_0 = Kp_SP_0 / Ti_SP_0; Kd_SP_0 = Kp_SP_0 * Td_SP_0;

% Mode 2: Set-point Tracking with 20% Overshoot
Kp_SP_20 = 0.95 * base_gain; Ti_SP_20 = 1.4 * Tau;   Td_SP_20 = 0.47 * L;
Ki_SP_20 = Kp_SP_20 / Ti_SP_20; Kd_SP_20 = Kp_SP_20 * Td_SP_20;

% Mode 3: Disturbance Rejection with 0% Overshoot
Kp_Dist_0 = 0.95 * base_gain; Ti_Dist_0 = 2.4 * L;   Td_Dist_0 = 0.42 * L;
Ki_Dist_0 = Kp_Dist_0 / Ti_Dist_0; Kd_Dist_0 = Kp_Dist_0 * Td_Dist_0;

% Mode 4: Disturbance Rejection with 20% Overshoot
Kp_Dist_20 = 1.2 * base_gain; Ti_Dist_20 = 2.4 * L;  Td_Dist_20 = 0.42 * L;
Ki_Dist_20 = Kp_Dist_20 / Ti_Dist_20; Kd_Dist_20 = Kp_Dist_20 * Td_Dist_20;

%% PART 3: Dynamic Simulation Execution
tspan = [0 5000]; 
x0 = [1; 45; 0; 0]; % Starts at ambient 45C as requested

% Run Mode 1
current_mode = 'CHR_RUN'; Kp_c = Kp_SP_0; Ki_c = Ki_SP_0; Kd_c = Kd_SP_0;
[t1, x1] = ode45(@tank_mimo_chr_core, tspan, x0);

% Run Mode 2
Kp_c = Kp_SP_20; Ki_c = Ki_SP_20; Kd_c = Kd_SP_20;
[t2, x2] = ode45(@tank_mimo_chr_core, tspan, x0);

% Run Mode 3
Kp_c = Kp_Dist_0; Ki_c = Ki_Dist_0; Kd_c = Kd_Dist_0;
[t3, x3] = ode45(@tank_mimo_chr_core, tspan, x0);

% Run Mode 4
Kp_c = Kp_Dist_20; Ki_c = Ki_Dist_20; Kd_c = Kd_Dist_20;
[t4, x4] = ode45(@tank_mimo_chr_core, tspan, x0);

%% PART 4: Plotting and Analysis Window
figure('Name', 'MIMO Tank - CHR Comprehensive Evaluation', 'NumberTitle', 'off');

plot(t1, x1(:,2), 'b', 'LineWidth', 1.5); hold on;
plot(t2, x2(:,2), 'r--', 'LineWidth', 1.5);
plot(t3, x3(:,2), 'g:', 'LineWidth', 2);
plot(t4, x4(:,2), 'm-.', 'LineWidth', 1.5);

% Target Reference Plot Line
t_ref = [0 500 500 5000]; T_ref = [45 45 50 50];
plot(t_ref, T_ref, 'k-.', 'LineWidth', 1);

grid on;
title('Nonlinear Temperature Dynamics Under Four CHR Criteria');
xlabel('Time (sec)'); ylabel('Temperature (\circC)');
legend('CHR SP (0% OS)', 'CHR SP (20% OS)', 'CHR Dist (0% OS)', 'CHR Dist (20% OS)', 'Reference SP');

%% PART 5: Unified Plant Core Dynamics
function dxdt = tank_mimo_chr_core(t, x)
    global A_tank kv Tin_nominal inv_rho_Cp h0 T0
    global Kp_level Ki_level Kp_temp Ki_temp D21
    global Kp_c Ki_c Kd_c
    
    h = x(1); T = x(2); int_err_h = x(3); int_err_T = x(4);   
    
    % Step Change at t=500s
    if t >= 500, T_sp = T0; else, T_sp = 45; end
    
    % Disturbance at t=2500s
    if t >= 2500, Tin = 20; else, Tin = Tin_nominal; end
    
    err_T = T_sp - T; err_h = h0 - h; 
    
    % Process state derivative approximation
    qin0_approx = 0.001; Qh0_approx = 104500;
    dT_dt_approx = (qin0_approx / (A_tank * h)) * (Tin - T) + (Qh0_approx * inv_rho_Cp / (A_tank * h));
    
    % Dynamic PID action based on current global assignments
    der_err_T = -dT_dt_approx;
    delta_u2_fb = Kp_c * err_T + Ki_c * int_err_T + Kd_c * der_err_T;
    
    % Decoupling and execution bounds
    delta_u1 = Kp_level * err_h + Ki_level * int_err_h;
    delta_u2_ff = D21 * delta_u1;
    
    qin = max(0, min(0.01, 0.001 + delta_u1)); 
    Qh = max(0, min(500000, 104500 + delta_u2_fb + delta_u2_ff));
    
    dh_dt = (1 / A_tank) * (qin - kv * sqrt(h));
    dT_dt = (qin / (A_tank * h)) * (Tin - T) + (Qh * inv_rho_Cp / (A_tank * h));
    
    dxdt = [dh_dt; dT_dt; err_h; err_T];
end