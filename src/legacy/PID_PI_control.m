
clear; clc; close all;

%% PART 1: System Constants & Baseline Control Gains
global A_tank kv Tin_nominal inv_rho_Cp h0 T0
global Kp_level Ki_level Kp_temp Ki_temp D21
global Kp_ZN_PI Ki_ZN_PI Kp_ZN_PID Ki_ZN_PID Kd_ZN_PID current_mode

A_tank = 0.785;       
kv = 0.001;           
Tin_nominal = 25;     
inv_rho_Cp = 2.392e-7;
h0 = 1; T0 =45;      

% Initial pole-placement controller gains
Kp_level = 0.013083;  Ki_level = 0.000008;
Kp_temp = 109392.419; Ki_temp = 139.353;
D21 = 1.045e08;

%% PART 2: Ziegler-Nichols Parameter Derivations
K_fr = 0.0002392;
Tau_fr = 785;
L_fr = 20; 

% Ziegler-Nichols PI Parameters
Kp_ZN_PI = 0.9 * (Tau_fr / (K_fr * L_fr));
Ki_ZN_PI = Kp_ZN_PI / (L_fr / 0.3);

% Ziegler-Nichols PID Parameters
Kp_ZN_PID = 1.2 * (Tau_fr / (K_fr * L_fr));
Ki_ZN_PID = Kp_ZN_PID / (2 * L_fr);
Kd_ZN_PID = Kp_ZN_PID * (0.5 * L_fr);

%% PART 3: Simulation Execution Over Core ZN Scenarios
tspan = [0 5000]; 
x0 = [1; 45; 0; 0]; 

% Scenario 1: Initial Baseline PI Control
current_mode = 'INITIAL_PI';
[t1, x1] = ode45(@tank_mimo_sequential_core, tspan, x0);

% Scenario 2: Ziegler-Nichols PI Control
current_mode = 'ZN_PI';
[t3, x3] = ode45(@tank_mimo_sequential_core, tspan, x0);

% Scenario 3: Ziegler-Nichols PID Control
current_mode = 'ZN_PID';
[t2, x2] = ode45(@tank_mimo_sequential_core, tspan, x0);

%% PART 4: Plotting Sequential Tracking and Disturbance Rejection
figure('Name', 'MIMO Tank - Step & Disturbance Analysis (ZN)', 'NumberTitle', 'off');

% Subplot 1: Tank Level Dynamics
subplot(2,1,1);
plot(t1, x1(:,1), 'b', 'LineWidth', 1.5); hold on;
plot(t3, x3(:,1), 'g:', 'LineWidth', 2);
plot(t2, x2(:,1), 'r--', 'LineWidth', 1.5);
grid on;
title('Nonlinear Tank Level Response (h)');
xlabel('Time (sec)'); ylabel('Height (m)');
legend('Initial PI', 'ZN-PI', 'ZN-PID');

% Subplot 2: Temperature Dynamics (Step + Disturbance)
subplot(2,1,2);
plot(t1, x1(:,2), 'b', 'LineWidth', 1.5); hold on;
plot(t3, x3(:,2), 'g:', 'LineWidth', 2);
plot(t2, x2(:,2), 'r--', 'LineWidth', 1.5);

% Plot the reference trajectory line
t_ref = [0 500 500 2500 2500 5000];
T_ref = [45 45  50  50   50   50];
plot(t_ref, T_ref, 'k-.', 'LineWidth', 1);

grid on;
title('Nonlinear Temperature Response: Step Input (t=500s) & Disturbance (t=2500s)');
xlabel('Time (sec)'); ylabel('Temperature (\circC)');
legend('Initial PI', 'ZN-PI', 'ZN-PID', 'Reference SP');

%% PART 5: Plant Dynamics and Sequential Control Logic
function dxdt = tank_mimo_sequential_core(t, x)
    global A_tank kv Tin_nominal inv_rho_Cp h0 T0
    global Kp_level Ki_level Kp_temp Ki_temp D21
    global Kp_ZN_PI Ki_ZN_PI Kp_ZN_PID Ki_ZN_PID Kd_ZN_PID current_mode
    
    h = x(1);           
    T = x(2);           
    int_err_h = x(3);   
    int_err_T = x(4);   
    
    % 1. Step Change in Set-point: T_sp jumps from 50 to 55 at t = 500s
    if t >= 500
        T_sp = 50;
    else
        T_sp = T0;
    end
    
    % 2. Inlet Temperature Disturbance: Tin drops from 25 to 20 at t = 2500s
    if t >= 2500
        Tin = 20; 
    else
        Tin = Tin_nominal;
    end
    
    err_T = T_sp - T;
    err_h = h0 - h; 
    
    % State derivative approximation for numerical integration of PID
    qin0_approx = 0.001; Qh0_approx = 104500;
    dT_dt_approx = (qin0_approx / (A_tank * h)) * (Tin - T) + (Qh0_approx * inv_rho_Cp / (A_tank * h));
    
    % Controller Switching
    switch current_mode
        case 'INITIAL_PI'
            delta_u2_fb = Kp_temp * err_T + Ki_temp * int_err_T;
            
        case 'ZN_PI'
            delta_u2_fb = Kp_ZN_PI * err_T + Ki_ZN_PI * int_err_T;
            
        case 'ZN_PID'
            der_err_T = -dT_dt_approx; 
            delta_u2_fb = Kp_ZN_PID * err_T + Ki_ZN_PID * int_err_T + Kd_ZN_PID * der_err_T;
    end
    
    % Decoupling and Actuator Limits
    delta_u1 = Kp_level * err_h + Ki_level * int_err_h;
    delta_u2_ff = D21 * delta_u1;
    
    qin = max(0, min(0.01, 0.001 + delta_u1)); 
    Qh = max(0, min(500000, 104500 + delta_u2_fb + delta_u2_ff));
    
    % Process Equations
    dh_dt = (1 / A_tank) * (qin - kv * sqrt(h));
    dT_dt = (qin / (A_tank * h)) * (Tin - T) + (Qh * inv_rho_Cp / (A_tank * h));
    
    d_int_err_h = err_h;
    d_int_err_T = err_T;
    
    dxdt = [dh_dt; dT_dt; d_int_err_h; d_int_err_T];
end