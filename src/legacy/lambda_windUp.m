
clear; clc; close all;

%% PART 1: System Constants & Baseline Setup
global A_tank kv Tin_nominal inv_rho_Cp h0 T0
global Kp_level Ki_level Kp_temp Ki_temp D21
global Kp_L Ki_L Kd_L Kw Aw_Active

A_tank = 0.785;       
kv = 0.001;           
Tin_nominal = 25;     
inv_rho_Cp = 2.392e-7;
h0 = 1; T0 = 50; 

% Decoupling Matrix Setup
Kp_level = 0.013083;  Ki_level = 0.000008;
Kp_temp = 109392.419; Ki_temp = 139.353;
D21 = 1.045e08;

K = 0.0002392; Tau = 785; L = 20;
tspan = [0 5000]; 
x0 = [1; 45; 0; 0]; % Init state [h, T, int_err_h, int_err_T]

%% PART 2: Lambda Controller Parameters Configuration
Lambda = Tau; % Original Conservative Case
Kp_L = (Tau + L/2) / (K * (Lambda + L/2));
Ti_L = Tau + L/2;
Td_L = (Tau * L) / (2*Tau + L);
Ki_L = Kp_L / Ti_L; 
Kd_L = Kp_L * Td_L;

% Anti-Windup Tracking Gain (Standard Back-Calculation)
Kw = 1 / Ti_L; 

%% PART 3: Run Simulation Without Anti-Windup
Aw_Active = 0; % Deactivate Anti-Windup
[t_raw, x_raw] = ode45(@tank_mimo_aw_core, tspan, x0);

% Post-processing to extract internal signals (Control Action & Error)
[u2_raw, err_raw] = extract_signals(t_raw, x_raw);

%% PART 4: Run Simulation With Anti-Windup Activated
Aw_Active = 1; % Activate Anti-Windup
[t_aw, x_aw] = ode45(@tank_mimo_aw_core, tspan, x0);

% Post-processing for Anti-Windup signals
[u2_aw, err_aw] = extract_signals(t_aw, x_aw);

%% PART 5: Plotting Performance & Saturation Dynamics

% --- FIGURE 1: Temperature Response Comparison ---
figure('Name', 'Temperature Analysis - Anti-Windup Integration', 'NumberTitle', 'off');
plot(t_raw, x_raw(:,2), 'r--', 'LineWidth', 1.5); hold on;
plot(t_aw, x_aw(:,2), 'b', 'LineWidth', 2);
t_ref = [0 500 500 5000]; T_ref = [45 45 50 50];
plot(t_ref, T_ref, 'k-.', 'LineWidth', 1);
grid on;
title('MIMO Tank Temperature Response with \lambda = \tau');
xlabel('Time (sec)'); ylabel('Temperature (\circC)');
legend('Without Anti-Windup (Windup Effect)', 'With Anti-Windup (Optimized)', 'Reference SP');

% --- FIGURE 2: Error and Actuator Saturation Limits ---
figure('Name', 'Actuator Saturation & Tracking Error Profile', 'NumberTitle', 'off');

% Subplot 1: Control Error (T_sp - T)
subplot(2,1,1);
plot(t_raw, err_raw, 'r--', 'LineWidth', 1.5); hold on;
plot(t_aw, err_aw, 'b', 'LineWidth', 1.5);
grid on;
title('Tracking Error Dynamic Trajectory (e(t))');
xlabel('Time (sec)'); ylabel('Error (\circC)');
legend('Error (Without AW)', 'Error (With AW)');

% Subplot 2: Actuator Output vs Saturation Bounds
subplot(2,1,2);
plot(t_raw, u2_raw, 'r--', 'LineWidth', 1.5); hold on;
plot(t_aw, u2_aw, 'b', 'LineWidth', 1.5);

% Plot Limits
line([0 5000], [500000 500000], 'Color', 'k', 'LineStyle', ':', 'LineWidth', 2);
line([0 5000], [0 0], 'Color', 'k', 'LineStyle', ':', 'LineWidth', 2);

grid on;
title('Heater Power Control Action (Q_h) vs Physical Saturation Bounds');
xlabel('Time (sec)'); ylabel('Power (Watts)');
ylim([-50000 600000]);
legend('Q_h Calculated (Without AW)', 'Q_h Calculated (With AW)', 'Saturation Limits (0 - 500kW)');

%% AUXILIARY FUNCTIONS: Plant Core and Signal Extractor
function dxdt = tank_mimo_aw_core(t, x)
    global A_tank kv Tin_nominal inv_rho_Cp h0 T0
    global Kp_level Ki_level Kp_temp Ki_temp D21
    global Kp_L Ki_L Kd_L Aw_Active
    
    h = x(1); T = x(2); int_err_h = x(3); int_err_T = x(4);   
    if t >= 500, T_sp = T0; else, T_sp = 45; end
    if t >= 2500, Tin = 20; else, Tin = Tin_nominal; end
    
    err_T = T_sp - T; err_h = h0 - h; 
    
    qin0_approx = 0.001; Qh0_approx = 104500;
    dT_dt_approx = (qin0_approx / (A_tank * h)) * (Tin - T) + (Qh0_approx * inv_rho_Cp / (A_tank * h));
    der_err_T = -dT_dt_approx;
    
    delta_u1 = Kp_level * err_h + Ki_level * int_err_h;
    delta_u2_ff = D21 * delta_u1;
    
    % محاسبه خروجی پی‌آی‌دی
    delta_u2_fb = Kp_L * err_T + Ki_L * int_err_T + Kd_L * der_err_T;
    Qh_calculated = 104500 + delta_u2_fb + delta_u2_ff;
    
    % منطق هوشمند Clamping Anti-Windup
    if Aw_Active == 1
        % اگر المنت در اشباع باشد و خطا هم‌جهت با اشباع باشد، انتگرال‌گیر متوقف می‌شود
        if (Qh_calculated >= 500000 && err_T > 0) || (Qh_calculated <= 0 && err_T < 0)
            d_int_err_T = 0; % متوقف کردن انتگرال‌گیر
        else
            d_int_err_T = err_T;
        end
    else
        d_int_err_T = err_T; % حالت معمولی و آسیب‌پذیر به وایندآپ
    end
    
    qin = max(0, min(0.01, 0.001 + delta_u1)); 
    dh_dt = (1 / A_tank) * (qin - kv * sqrt(h));
    
    % اعمال اشباع واقعی روی فیزیک فرآیند مخزن
    Qh_saturated = max(0, min(500000, Qh_calculated));
    dT_dt = (qin / (A_tank * h)) * (Tin - T) + (Qh_saturated * inv_rho_Cp / (A_tank * h));
    
    dxdt = [dh_dt; dT_dt; err_h; d_int_err_T];
end

function [u2_vec, err_vec] = extract_signals(t_vec, x_vec)
    global Kp_level Ki_level Kp_temp Ki_temp D21 Kp_L Ki_L Kd_L T0 h0 A_tank Tin_nominal inv_rho_Cp
    u2_vec = zeros(length(t_vec),1);
    err_vec = zeros(length(t_vec),1);
    for i = 1:length(t_vec)
        t = t_vec(i); h = x_vec(i,1); T = x_vec(i,2); int_err_h = x_vec(i,3); int_err_T = x_vec(i,4);
        if t >= 500, T_sp = T0; else, T_sp = 45; end
        if t >= 2500, Tin = 20; else, Tin = Tin_nominal; end
        err_T = T_sp - T; err_h = h0 - h;
        err_vec(i) = err_T;
        
        qin0_approx = 0.001; Qh0_approx = 104500;
        dT_dt_approx = (qin0_approx / (A_tank * h)) * (Tin - T) + (Qh0_approx * inv_rho_Cp / (A_tank * h));
        der_err_T = -dT_dt_approx;
        
        delta_u1 = Kp_level * err_h + Ki_level * int_err_h;
        delta_u2_ff = D21 * delta_u1;
        delta_u2_fb = Kp_L * err_T + Ki_L * int_err_T + Kd_L * der_err_T;
        
        % این بار مقدار واقعی و فیزیکی اعمال شده به المنت را رسم می‌کنیم تا مرزها مشخص شوند
        u2_vec(i) = max(0, min(500000, 104500 + delta_u2_fb + delta_u2_ff));
    end
end