function dxdt = nonlinear_tank_model(t, x, params, controller, options)
%NONLINEAR_TANK_MODEL Closed-loop nonlinear MIMO tank dynamics.
%
% State vector:
%   x(1) = tank level h [m]
%   x(2) = tank temperature T [degC]
%   x(3) = integral of level error
%   x(4) = integral of temperature error

arguments
    t double
    x double
    params struct
    controller struct
    options.antiWindup logical = false
end

h = max(x(1), 1e-6);       % Avoid division by zero in thermal dynamics
T = x(2);
int_err_h = x(3);
int_err_T = x(4);

if t >= params.setpoint_step_time
    T_sp = params.T_setpoint_after_step;
else
    T_sp = params.T_setpoint_before_step;
end

if t >= params.disturbance_time
    Tin = params.Tin_disturbed;
else
    Tin = params.Tin_nominal;
end

err_h = params.h0 - h;
err_T = T_sp - T;

% Level loop and feedforward decoupling
Delta_qin = params.Kp_level * err_h + params.Ki_level * int_err_h;
Delta_Qh_ff = params.D21 * Delta_qin;

% Approximate derivative term for the temperature error
nominal_dTdt = (params.qin0 / (params.A_tank * h)) * (Tin - T) + ...
               (params.Qh0 * params.inv_rho_Cp / (params.A_tank * h));
der_err_T = -nominal_dTdt;

Delta_Qh_fb = controller.Kp * err_T + ...
              controller.Ki * int_err_T + ...
              controller.Kd * der_err_T;

Qh_unsaturated = params.Qh0 + Delta_Qh_fb + Delta_Qh_ff;
qin = saturate(params.qin0 + Delta_qin, params.qin_min, params.qin_max);
Qh = saturate(Qh_unsaturated, params.Qh_min, params.Qh_max);

% Optional clamping anti-windup for heater saturation
if options.antiWindup && ((Qh_unsaturated >= params.Qh_max && err_T > 0) || ...
                          (Qh_unsaturated <= params.Qh_min && err_T < 0))
    d_int_err_T = 0;
else
    d_int_err_T = err_T;
end

dhdt = (qin - params.kv * sqrt(h)) / params.A_tank;
dTdt = (qin / (params.A_tank * h)) * (Tin - T) + ...
       (Qh * params.inv_rho_Cp / (params.A_tank * h));

dxdt = [dhdt; dTdt; err_h; d_int_err_T];
end

function y = saturate(u, lowerBound, upperBound)
y = max(lowerBound, min(upperBound, u));
end
