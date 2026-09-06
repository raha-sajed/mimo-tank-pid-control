function sim = simulate_closed_loop(params, controller, options)
%SIMULATE_CLOSED_LOOP Simulate the closed-loop nonlinear tank model.

arguments
    params struct
    controller struct
    options.antiWindup logical = false
end

odefun = @(t, x) nonlinear_tank_model(t, x, params, controller, ...
                                      antiWindup=options.antiWindup);
[t, x] = ode45(odefun, params.tspan, params.initial_state);

[u, eT] = reconstruct_control_signals(t, x, params, controller);

sim = struct();
sim.t = t;
sim.x = x;
sim.h = x(:, 1);
sim.T = x(:, 2);
sim.Qh = u.Qh;
sim.qin = u.qin;
sim.temperatureError = eT;
sim.controller = controller;
sim.antiWindup = options.antiWindup;
end

function [u, eT] = reconstruct_control_signals(t, x, params, controller)
n = numel(t);
Qh = zeros(n, 1);
qin = zeros(n, 1);
eT = zeros(n, 1);

for i = 1:n
    h = max(x(i, 1), 1e-6);
    T = x(i, 2);
    int_err_h = x(i, 3);
    int_err_T = x(i, 4);

    if t(i) >= params.setpoint_step_time
        T_sp = params.T_setpoint_after_step;
    else
        T_sp = params.T_setpoint_before_step;
    end

    if t(i) >= params.disturbance_time
        Tin = params.Tin_disturbed;
    else
        Tin = params.Tin_nominal;
    end

    err_h = params.h0 - h;
    eT(i) = T_sp - T;

    Delta_qin = params.Kp_level * err_h + params.Ki_level * int_err_h;
    Delta_Qh_ff = params.D21 * Delta_qin;

    nominal_dTdt = (params.qin0 / (params.A_tank * h)) * (Tin - T) + ...
                   (params.Qh0 * params.inv_rho_Cp / (params.A_tank * h));
    der_err_T = -nominal_dTdt;

    Delta_Qh_fb = controller.Kp * eT(i) + ...
                  controller.Ki * int_err_T + ...
                  controller.Kd * der_err_T;

    qin(i) = max(params.qin_min, min(params.qin_max, params.qin0 + Delta_qin));
    Qh(i) = max(params.Qh_min, min(params.Qh_max, params.Qh0 + Delta_Qh_fb + Delta_Qh_ff));
end

u = struct('qin', qin, 'Qh', Qh);
end
