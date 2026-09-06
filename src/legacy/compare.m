% Methods: ZN, CHR, Cohen-Coon, Lambda, Maximally Flat, Haalman, AMIGO
% Author: Fatemeh Sadat Sajed
clear; clc; close all;

%% PART 1: System Constants & Baseline Setup
global A_tank kv Tin_nominal inv_rho_Cp h0 T0
global Kp_level Ki_level Kp_temp Ki_temp D21
global Kp_current Ki_current Kd_current

A_tank = 0.785;       % Tank cross-sectional area (m^2)
kv = 0.001;           % Valve discharge coefficient
Tin_nominal = 25;     % Nominal inlet temperature (C)
inv_rho_Cp = 2.392e-7;% Reciprocal of density times specific heat capacity
h0 = 1; T0 = 50;      % Operating points

% Level loop and Decoupling parameters
Kp_level = 0.013083;  Ki_level = 0.000008;
D21 = 1.045e08;

% FOPTD Plant Parameters
K = 0.0002392; Tau = 785; L = 20;
tspan = [0 5000]; 
x0 = [1; 45; 0; 0]; % Init state vector

%% PART 2: PID Gains Repository for All Methods
methods_list = {'Ziegler-Nichols', 'CHR (0% OS)', 'Cohen-Coon', ...
                'Lambda (\lambda=3\timesL)', 'Maximally Flat', 'Haalman', 'AMIGO'};
            
num_methods = length(methods_list);
Kp_arr = zeros(num_methods, 1);
Ki_arr = zeros(num_methods, 1);
Kd_arr = zeros(num_methods, 1);

% 1. Ziegler-Nichols (Standard Step Response)
Kp_arr(1) = (1.2 * Tau) / (K * L);
Ti_ZN = 2 * L; Td_ZN = 0.5 * L;
Ki_arr(1) = Kp_arr(1) / Ti_ZN; Kd_arr(1) = Kp_arr(1) * Td_ZN;

% 2. CHR (0% Overshoot Setpoint Regulation)
Kp_arr(2) = (0.6 * Tau) / (K * L);
Ti_CHR = Tau; Td_CHR = 0.5 * L;
Ki_arr(2) = Kp_arr(2) / Ti_CHR; Kd_arr(2) = Kp_arr(2) * Td_CHR;

% 3. Cohen-Coon
Kp_arr(3) = (Tau / (K * L)) * (4/3 + L / (4 * Tau));
Ti_CC = L * ((32 + 6 * (L / Tau)) / (13 + 8 * (L / Tau)));
Td_CC = L * (4 / (11 + 2 * (L / Tau)));
Ki_arr(3) = Kp_arr(3) / Ti_CC; Kd_arr(3) = Kp_arr(3) * Td_CC;

% 4. Lambda Tuning (Optimized Lambda = 3*L)
Lambda = 3 * L; % Complies with the rule Lambda > 3*L (60 seconds)
Kp_arr(4) = (Tau + L/2) / (K * (Lambda + L/2));
Ti_L = Tau + L/2; 
Td_L = (Tau * L) / (2 * Tau + L);
Ki_arr(4) = Kp_arr(4) / Ti_L; 
Kd_arr(4) = Kp_arr(4) * Td_L;

% 5. Maximally Flat
Kp_arr(5) = Tau / (K * L * exp(1));
Ti_MF = Tau; Td_MF = L / 2;
Ki_arr(5) = Kp_arr(5) / Ti_MF; Kd_arr(5) = Kp_arr(5) * Td_MF;

% 6. Haalman
Kp_arr(6) = Tau / (2 * K * L);
Ti_Hl = Tau; Td_Hl = L / 2;
Ki_arr(6) = Kp_arr(6) / Ti_Hl; Kd_arr(6) = Kp_arr(6) * Td_Hl;

% 7. AMIGO (Åström-Häglund Optimization)
Kp_arr(7) = (1 / K) * (0.25 + 0.45 * (Tau / L));
Ti_AMIGO = L * ((0.4 + 1.2 * (Tau / L)) / (1 + 0.05 * (Tau / L)));
Td_AMIGO = L * (0.5 / (1 + 0.3 * (Tau / L)));
Ki_arr(7) = Kp_arr(7) / Ti_AMIGO; Kd_arr(7) = Kp_arr(7) * Td_AMIGO;

%% PART 3: Multi-Run Simulation Loop
t_cell = cell(num_methods, 1);
x_cell = cell(num_methods, 1);
u_cell = cell(num_methods, 1);

for idx = 1:num_methods
    % Dynamically update the active controller parameters
    Kp_current = Kp_arr(idx);
    Ki_current = Ki_arr(idx);
    Kd_current = Kd_arr(idx);
    
    % Execute Simulation
    [t_out, x_out] = ode45(@tank_mimo_comprehensive_core, tspan, x0);
    
    t_cell{idx} = t_out;
    x_cell{idx} = x_out;
    
    % Post-process to extract Actuator Signal Profile
    u_cell{idx} = extract_actuator_signal(t_out, x_out);
end

%% PART 4: Unified Graphical Visualization
colors = {'r', 'g', 'b', 'k', 'c', 'm', [0.85 0.33 0.1]};

% --- Figure 1: Comprehensive Temperature Comparison ---
figure('Name', 'Unified Temperature Analysis', 'NumberTitle', 'off');
for idx = 1:num_methods
    plot(t_cell{idx}, x_cell{idx}(:,2), 'Color', colors{idx}, 'LineWidth', 1.8); hold on;
end
t_ref = [0 500 500 5000]; T_ref = [45 45 50 50];
plot(t_ref, T_ref, 'r--', 'LineWidth', 1.2);
grid on; title('MIMO Tank Temperature Response: Master Comparison');
xlabel('Time (sec)'); ylabel('Temperature (\circC)');
legend([methods_list, {'Reference SP'}], 'Location', 'best');

% --- Figure 2: Comprehensive Control Action Profile ---
figure('Name', 'Unified Actuator Signal Analysis', 'NumberTitle', 'off');
for idx = 1:num_methods
    plot(t_cell{idx}, u_cell{idx}, 'Color', colors{idx}, 'LineWidth', 1.5); hold on;
end
grid on; title('Heater Power Control Action (Q_h): Master Comparison');
xlabel('Time (sec)'); ylabel('Power (Watts)');
legend(methods_list, 'Location', 'best');

%% PART 5: Shared Process Differential Equations Core
function dxdt = tank_mimo_comprehensive_core(t, x)
    global A_tank kv Tin_nominal inv_rho_Cp h0 T0
    global Kp_level Ki_level D21
    global Kp_current Ki_current Kd_current
    
    h = x(1); T = x(2); int_err_h = x(3); int_err_T = x(4);   
    
    if t >= 500, T_sp = T0; else, T_sp = 45; end
    if t >= 2500, Tin = 20; else, Tin = Tin_nominal; end
    
    err_T = T_sp - T; err_h = h0 - h; 
    
    qin0_approx = 0.001; Qh0_approx = 104500;
    dT_dt_approx = (qin0_approx / (A_tank * h)) * (Tin - T) + (Qh0_approx * inv_rho_Cp / (A_tank * h));
    der_err_T = -dT_dt_approx;
    
    % Core Parallel PID Implementation
    delta_u2_fb = Kp_current * err_T + Ki_current * int_err_T + Kd_current * der_err_T;
    
    delta_u1 = Kp_level * err_h + Ki_level * int_err_h;
    delta_u2_ff = D21 * delta_u1;
    
    qin = max(0, min(0.01, 0.001 + delta_u1)); 
    Qh = max(0, min(500000, 104500 + delta_u2_fb + delta_u2_ff));
    
    dh_dt = (1 / A_tank) * (qin - kv * sqrt(h));
    dT_dt = (qin / (A_tank * h)) * (Tin - T) + (Qh * inv_rho_Cp / (A_tank * h));
    
    dxdt = [dh_dt; dT_dt; err_h; err_T];
end

function Qh_vec = extract_actuator_signal(t_vec, x_vec)
    global Kp_level Ki_level D21 Kp_current Ki_current Kd_current T0 h0 A_tank Tin_nominal inv_rho_Cp
    Qh_vec = zeros(length(t_vec), 1);
    for i = 1:length(t_vec)
        t = t_vec(i); h = x_vec(i,1); T = x_vec(i,2); int_err_h = x_vec(i,3); int_err_T = x_vec(i,4);
        if t >= 500, T_sp = T0; else, T_sp = 45; end
        if t >= 2500, Tin = 20; else, Tin = Tin_nominal; end
        err_T = T_sp - T; err_h = h0 - h;
        
        qin0_approx = 0.001; Qh0_approx = 104500;
        dT_dt_approx = (qin0_approx / (A_tank * h)) * (Tin - T) + (Qh0_approx * inv_rho_Cp / (A_tank * h));
        der_err_T = -dT_dt_approx;
        
        delta_u2_fb = Kp_current * err_T + Ki_current * int_err_T + Kd_current * der_err_T;
        delta_u1 = Kp_level * err_h + Ki_level * int_err_h;
        delta_u2_ff = D21 * delta_u1;
        
        Qh_vec(i) = max(0, min(500000, 104500 + delta_u2_fb + delta_u2_ff));
    end
end