
clear; clc; close all;

%% PART 1: System Constants & Baseline Setup
global A_tank kv Tin_nominal inv_rho_Cp h0 T0
global Kp_level Ki_level Kp_temp Ki_temp D21
global Kp_AMIGO Ki_AMIGO Kd_AMIGO

A_tank = 0.785;       % Tank cross-sectional area (m^2)
kv = 0.001;           % Valve discharge coefficient
Tin_nominal = 25;     % Nominal inlet temperature (C)
inv_rho_Cp = 2.392e-7;% Reciprocal of density times specific heat capacity
h0 = 1; T0 = 50;      % Nominal operating points

% Decentralized and decoupling loops configuration
Kp_level = 0.013083;  Ki_level = 0.000008;
Kp_temp = 109392.419; Ki_temp = 139.353;
D21 = 1.045e08;       % Cross-decoupler feedforward gain

%% PART 2: AMIGO Tuning Parameter Calculations
K = 0.0002392; Tau = 785; L = 20;

% Åström-Häglund AMIGO standard optimization formulas
Kp_AMIGO = (1 / K) * (0.25 + 0.45 * (Tau / L));
Ti_AMIGO = L * ((0.4 + 1.2 * (Tau / L)) / (1 + 0.05 * (Tau / L)));
Td_AMIGO = L * (0.5 / (1 + 0.3 * (Tau / L)));

% Convert standard time constants to parallel PID controller gains
Ki_AMIGO = Kp_AMIGO / Ti_AMIGO;
Kd_AMIGO = Kp_AMIGO * Td_AMIGO;

%% PART 3: Dynamic Simulation Execution
tspan = [0 5000]; 
x0 = [1; 45; 0; 0]; % Initial state vector (starts at T = 45C)

% Solve nonlinear differential equations using variable-step ODE45 solver
[t, x] = ode45(@tank_mimo_amigo_core, tspan, x0);

%% PART 4: Plotting the AMIGO Performance
figure('Name', 'MIMO Tank - AMIGO Evaluation', 'NumberTitle', 'off');

% Plot nonlinear tank temperature response
plot(t, x(:,2), 'm', 'LineWidth', 2); hold on;

% Plot reference set-point line
t_ref = [0 500 500 5000]; T_ref = [45 45 50 50];
plot(t_ref, T_ref, 'k-.', 'LineWidth', 1);

grid on;
title('Nonlinear Temperature Dynamics Under AMIGO PID Tuning');
xlabel('Time (sec)'); ylabel('Temperature (\circC)');
legend('AMIGO PID', 'Reference SP');

%% PART 5: Plant Core Dynamics Function
function dxdt = tank_mimo_amigo_core(t, x)
    global A_tank kv Tin_nominal inv_rho_Cp h0 T0
    global Kp_level Ki_level Kp_temp Ki_temp D21
    global Kp_AMIGO Ki_AMIGO Kd_AMIGO
    
    h = x(1); T = x(2); int_err_h = x(3); int_err_T = x(4);   
    
    % Step Change implementation at t = 500s
    if t >= 500, T_sp = T0; else, T_sp = 45; end
    
    % Load Disturbance introduction at t = 2500s
    if t >= 2500, Tin = 20; else, Tin = Tin_nominal; end
    
    err_T = T_sp - T; err_h = h0 - h; 
    
    % State derivative approximation for numerical integration stability
    qin0_approx = 0.001; Qh0_approx = 104500;
    dT_dt_approx = (qin0_approx / (A_tank * h)) * (Tin - T) + (Qh0_approx * inv_rho_Cp / (A_tank * h));
    
    % Feedback control law utilizing AMIGO parameters
    der_err_T = -dT_dt_approx;
    delta_u2_fb = Kp_AMIGO * err_T + Ki_AMIGO * int_err_T + Kd_AMIGO * der_err_T;
    
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