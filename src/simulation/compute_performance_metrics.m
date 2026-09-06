function metrics = compute_performance_metrics(sim, params)
%COMPUTE_PERFORMANCE_METRICS Estimate key temperature-response metrics.

T = sim.T;
t = sim.t;
finalSetpoint = params.T_setpoint_after_step;
stepIdx = t >= params.setpoint_step_time;
distIdx = t >= params.disturbance_time;

T_after_step = T(stepIdx);
t_after_step = t(stepIdx);

peakTemperature = max(T_after_step);
overshoot = max(0, peakTemperature - finalSetpoint);
minimumAfterDisturbance = min(T(distIdx));
disturbanceDip = finalSetpoint - minimumAfterDisturbance;

% Settling time: first time after step where response remains within +/-2% of step size.
stepAmplitude = abs(params.T_setpoint_after_step - params.T_setpoint_before_step);
tolerance = 0.02 * stepAmplitude;
settlingTime = NaN;
for k = 1:numel(t_after_step)
    if all(abs(T_after_step(k:end) - finalSetpoint) <= tolerance)
        settlingTime = t_after_step(k) - params.setpoint_step_time;
        break;
    end
end

metrics = struct();
metrics.controller = string(sim.controller.name);
metrics.peak_temperature_c = peakTemperature;
metrics.overshoot_c = overshoot;
metrics.disturbance_minimum_c = minimumAfterDisturbance;
metrics.disturbance_dip_c = disturbanceDip;
metrics.settling_time_s = settlingTime;
metrics.final_temperature_c = T(end);
end
