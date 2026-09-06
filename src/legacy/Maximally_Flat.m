
clear; clc; close all;

%% PART 1: System Constants & Baseline Setup
global A_tank kv Tin_nominal inv_rho_Cp h0 T0
global Kp_level Ki_level Kp_temp Ki_temp D21
global Kp_MF Ki_MF Kd_MF

A_tank = 0.785;       % Tank cross-sectional area (m^2)
kv = 0.001;           % Valve discharge coefficient
Tin_nominal = 25;     % Nominal inlet temperature (C)
inv_rho_Cp = 2.392e-7;% Reciprocal of density times specific heat capacity
h0 = 1; T0 = 50;      % Nominal operating points for level and temperature

% Decentralized and decoupling loops configuration
Kp_level = 0.013083;  Ki_level = 0.000008;
Kp_temp = 109392.419; Ki_temp = 139.353;
D21 = 1.045e08;       % Cross-decoupler gain

%% PART 2: Maximally Flat Tuning Parameter Calculations
K = 0.0002392; Tau = 785; L = 20;

% Analytic formulas for Maximally Flat tuning technique
Kp_MF = Tau / (K * L * exp(1));
Ti_MF = Tau;
Td_MF = L / 2;

% Convert standard time constants to parallel PID controller gains
Ki_MF = Kp_MF / Ti_MF;
Kd_MF = Kp_MF * Td_MF;

%% PART 3: Dynamic Simulation Execution
tspan = [0 5000]; 
x0 = [1; 45; 0; 0]; % Initial state vector (starts at T = 45C)

% Solve nonlinear differential equations using variable-step ODE45 solver
[t, x] = ode45(@tank_mimo_max_flat_core, tspan, x0);

%% PART 4: Plotting the Maximally Flat Performance
figure('Name', 'MIMO Tank - Maximally Flat Evaluation', 'NumberTitle', 'off');

% Plot nonlinear tank temperature response
plot(t, x(:,2), 'b', 'LineWidth', 2); hold on;

% Plot reference set-point line
t_ref = [0 500 500 5000]; T_ref = [45 45 50 50];
plot(t_ref, T_ref, 'k-.', 'LineWidth', 1);

grid on;
title('Nonlinear Temperature Dynamics Under Maximally Flat PID Tuning');
xlabel('Time (sec)'); ylabel('Temperature (\circC)');
legend('Maximally Flat PID', 'Reference SP');

%% PART 5: Plant Core Dynamics Function
function dxdt = tank_mimo_max_flat_core(t, x)
    global A_tank kv Tin_nominal inv_rho_Cp h0 T0
    global Kp_level Ki_level Kp_temp Ki_temp D21
    global Kp_MF Ki_MF Kd_MF
    
    h = x(1); T = x(2); int_err_h = x(3); int_err_T = x(4);   
    
    % Step Change implementation at t = 500s
    if t >= 500, T_sp = T0; else, T_sp = 45; end
    
    % Load Disturbance introduction at t = 2500s
    if t >= 2500, Tin = 20; else, Tin = Tin_nominal; end
    
    err_T = T_sp - T; err_h = h0 - h; 
    
    % State derivative approximation for numerical integration stability
    qin0_approx = 0.001; Qh0_approx = 104500;
    dT_dt_approx = (qin0_approx / (A_tank * h)) * (Tin - T) + (Qh0_approx * inv_rho_Cp / (A_tank * h));
    
    % Feedback control law utilizing Maximally Flat parameters
    der_err_T = -dT_dt_approx;
    delta_u2_fb = Kp_MF * err_T + Ki_MF * int_err_T + Kd_MF * der_err_T;
    
    % Decoupling feedforward interaction calculator
    delta_u1 = Kp_level * err_h + Ki_level * int_err_h;
    delta_u2_ff = D21 * delta_u1;
    
    % Actuator saturation handling limits
    qin = max(0, min(0.01, 0.001 + delta_u1)); 
    Qh = max(0, min(500000, 104500 + delta_u2_fb + delta_u2_ff));
    
    % Evaluation of nonlinear process differential equations
    dh_dt = (1 / A_tank) * (qin - kv * sqrt(h));
    dT_dt = (qin / (A_tank * h)) * (Tin - T) + (Qh * inv_rho_Cp / (A_tank * h));
    
    dxdt = [dh_dt; dT_dt; err_h; err_T];
end