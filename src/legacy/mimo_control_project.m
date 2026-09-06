% Industrial Control Project - MIMO Tank Level and Temperature Control
% Author:  Raha Sajed
clear; clc; close all;

%% Step 1: System Parameters & Operating Points
A_tank = 0.785;       % Tank cross-sectional area (m^2)
h0 = 1;               % Operating height (m)
T0 = 50;              % Operating temperature (C)
Tin = 25;             % Inlet water temperature (C)
qin0 = 0.001;         % Operating inlet flow rate (m^3/s)
kv = 0.001;           % Valve constant
inv_rho_Cp = 2.392e-7;% 1 / (rho * Cp)

%% Step 2: Linear Transfer Functions (Continuous Time)
s = tf('s');

G11 = 2000 / (1570*s + 1);
G12 = 0;
G21 = -25000 / (785*s + 1);
G22 = 0.0002392 / (785*s + 1);

G_plant = [G11, G12; G21, G22];

fprintf('--- Open-Loop Transfer Function Matrix G(s) ---\n');
display(G_plant);

%% Step 3: Advanced Decoupler Design (Feedforward Decoupling)
% Since G12 = 0 and G21 ~= 0, we have a one-way coupling.
% We design a decoupler D21(s) to cancel the effect of delta_qin on delta_T.
% Condition: G21(s) + G22(s)*D21(s) = 0  =>  D21(s) = -G21(s) / G22(s)

D21 = - G21 / G22;
D21 = minreal(D21); % Simplify the transfer function

fprintf('--- Designed Decoupler D21(s) ---\n');
display(D21);

%% Step 4: Controller Tuning (PI/PID Design)
% Loop 1: Level Control (G11) using a PI Controller
% Desired closed-loop settling time ~ 300 seconds, no overshoot
Target_Tau1 = 60; 
Kp_level = 1570 / (2000 * Target_Tau1);
Ki_level = 1 / (2000 * Target_Tau1);
C_level = Kp_level + Ki_level/s;

% Loop 2: Temperature Control (G22) using a PI Controller
% Desired closed-loop settling time ~ 150 seconds
Target_Tau2 = 30;
Kp_temp = 785 / (0.0002392 * Target_Tau2);
Ki_temp = 1 / (0.0002392 * Target_Tau2);
C_temp = Kp_temp + Ki_temp/s;

fprintf('--- Controller Parameters ---\n');
fprintf('Level Controller: Kp = %f, Ki = %f\n', Kp_level, Ki_level);
fprintf('Temp Controller:  Kp = %f, Ki = %f\n', Kp_temp, Ki_temp);

%% Step 5: Closed-Loop Simulation (Linear Ideal Case)
% Ideal decoupled complementary sensitivity functions
T_level = minreal((C_level*G11) / (1 + C_level*G11));
T_temp = minreal((C_temp*G22) / (1 + C_temp*G22));

% Plot Linear Closed-Loop Step Response
figure('Name', 'Closed-Loop Ideal Response', 'NumberTitle', 'off');
subplot(2,1,1);
step(T_level, 500);
grid on;
title('Linear Closed-Loop Step Response: Level (h)');
xlabel('Time (sec)'); ylabel('Delta h (m)');

subplot(2,1,2);
step(T_temp, 500);
grid on;
title('Linear Closed-Loop Step Response: Temperature (T)');
xlabel('Time (sec)'); ylabel('Delta T (C)');


%% Step 6: Define Plant with Disturbance Input
% New Input Vector: U = [Delta_qin; Delta_Qh; Delta_Tin]
% Transfer function from Delta_Tin to Delta_h is 0
% Transfer function from Delta_Tin to Delta_T is G23(s) = qin0 / (A_tank*h0*s + qin0)

G23 = qin0 / (A_tank * h0 * s + qin0);

% Closed-loop response to a step disturbance in Tin (e.g., Delta_Tin = -5 Degrees)
% We analyze how the Temperature Controller (C_temp) rejects this disturbance
T_dist_to_Temp = minreal(G23 / (1 + C_temp * G22));

%% Step 7: Simulate and Plot Disturbance Response
t_sim = 0:1:5000;
dist_amplitude = -5; % Inlet water suddenly drops by 5 degrees Celsius
[y_dist, t_out] = step(T_dist_to_Temp, t_sim);

figure('Name', 'Disturbance Rejection Analysis', 'NumberTitle', 'off');
plot(t_out, y_dist * dist_amplitude, 'r', 'LineWidth', 2);
grid on;
title('Temperature Response to a -5^{\circ}C Step Disturbance in Tin');
xlabel('Time (sec)');
ylabel('Delta T (^{\circ}C)');
legend('Temperature Error (\Delta T)');