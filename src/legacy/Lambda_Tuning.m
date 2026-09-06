% Industrial Control Project - Lambda Tuning PID Implementation
% Author: Fatemeh Sadat Sajed
clear; clc; close all;

%% PART 1: System Constants & Baseline Setup
global A_tank kv Tin_nominal inv_rho_Cp h0 T0
global Kp_level Ki_level Kp_temp Ki_temp D21
global Kp_Lmb Ki_Lmb Kd_Lmb

A_tank = 0.785;       
kv = 0.001;           
Tin_nominal = 25;     
inv_rho_Cp = 2.392e-7;
h0 = 1; T0 = 50; 

% Decentralized and decoupling loops
Kp_level = 0.013083;  Ki_level = 0.000008;
Kp_temp = 109392.419; Ki_temp = 139.353;
D21 = 1.045e08;

%% PART 2: Lambda Tuning Calculations
K = 0.0002392; Tau = 785; L = 20;

% Choosing Lambda equal to Tau for smooth and robust response
Lambda = 2*L; 

% Lambda Tuning PID Analytic Formulas
Kp_Lmb = (Tau + L/2) / (K * (Lambda + L/2));
Ti_Lmb = Tau + L/2;
Td_Lmb = (Tau * L) / (2*Tau + L);

% Standard Controller Gains Conversion
Ki_Lmb = Kp_Lmb / Ti_Lmb;
Kd_Lmb = Kp_Lmb * Td_Lmb;

%% PART 3: Dynamic Simulation Execution
tspan = [0 5000]; 
x0 = [1; 45; 0; 0]; % Starts at 45C

[t, x] = ode45(@tank_mimo_lambda_core, tspan, x0);

%% PART 4: Plotting the Lambda Controller Performance
figure('Name', 'MIMO Tank - Lambda Tuning Evaluation', 'NumberTitle', 'off');

% Tank Temperature Plot
plot(t, x(:,2), 'm', 'LineWidth', 2); hold on;

% Target Reference Line
t_ref = [0 500 500 5000]; T_ref = [45 45 50 50];
plot(t_ref, T_ref, 'k-.', 'LineWidth', 1);

grid on;
title('Nonlinear Temperature Dynamics Under Lambda PID Tuning');
xlabel('Time (sec)'); ylabel('Temperature (\circC)');
legend('Lambda PID', 'Reference SP');

%% PART 5: Plant Core Dynamics
function dxdt = tank_mimo_lambda_core(t, x)
    global A_tank kv Tin_nominal inv_rho_Cp h0 T0
    global Kp_level Ki_level Kp_temp Ki_temp D21
    global Kp_Lmb Ki_Lmb Kd_Lmb
    
    h = x(1); T = x(2); int_err_h = x(3); int_err_T = x(4);   
    
    % Step Change at t=500s
    if t >= 500, T_sp = T0; else, T_sp = 45; end
    
    % Disturbance at t=2500s
    if t >= 2500, Tin = 20; else, Tin = Tin_nominal; end
    
    err_T = T_sp - T; err_h = h0 - h; 
    
    % State derivative approximation for numerical Integration
    qin0_approx = 0.001; Qh0_approx = 104500;
    dT_dt_approx = (qin0_approx / (A_tank * h)) * (Tin - T) + (Qh0_approx * inv_rho_Cp / (A_tank * h));
    
    % Continuous Lambda PID Feedback law
    der_err_T = -dT_dt_approx;
    delta_u2_fb = Kp_Lmb * err_T + Ki_Lmb * int_err_T + Kd_Lmb * der_err_T;
    
    % Decoupler and saturation handler
    delta_u1 = Kp_level * err_h + Ki_level * int_err_h;
    delta_u2_ff = D21 * delta_u1;
    
    qin = max(0, min(0.01, 0.001 + delta_u1)); 
    Qh = max(0, min(500000, 104500 + delta_u2_fb + delta_u2_ff));
    
    dh_dt = (1 / A_tank) * (qin - kv * sqrt(h));
    dT_dt = (qin / (A_tank * h)) * (Tin - T) + (Qh * inv_rho_Cp / (A_tank * h));
    
    dxdt = [dh_dt; dT_dt; err_h; err_T];
end