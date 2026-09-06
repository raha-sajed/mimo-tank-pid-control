function controller = compute_pid_gains(method, params)
%COMPUTE_PID_GAINS Compute PID gains for a selected tuning method.
%
% Inputs:
%   method - string/char specifying the tuning method
%   params - tank/process parameter structure from tank_parameters()
%
% Output:
%   controller - structure with Kp, Ki, Kd, name, and notes fields

arguments
    method {mustBeTextScalar}
    params struct
end

K = params.K;
Tau = params.Tau;
L = params.L;
methodKey = lower(strrep(strrep(string(method), " ", "_"), "-", "_"));

controller = struct();
controller.name = char(method);
controller.Kp = 0;
controller.Ki = 0;
controller.Kd = 0;
controller.notes = "";

switch methodKey
    case {"ziegler_nichols", "zn", "zn_pid"}
        controller.name = 'Ziegler-Nichols PID';
        controller.Kp = 1.2 * Tau / (K * L);
        Ti = 2 * L;
        Td = 0.5 * L;
        controller.notes = "Standard Ziegler-Nichols PID tuning.";

    case {"chr", "chr_0_os", "chr_setpoint_0_os"}
        controller.name = 'CHR 0% Overshoot';
        controller.Kp = 0.6 * Tau / (K * L);
        Ti = Tau;
        Td = 0.5 * L;
        controller.notes = "CHR setpoint-tracking tuning with 0% nominal overshoot.";

    case {"cohen_coon", "cohencoon"}
        controller.name = 'Cohen-Coon';
        r = L / Tau;
        controller.Kp = (Tau / (K * L)) * ((4/3) + (r/4));
        Ti = L * ((32 + 6*r) / (13 + 8*r));
        Td = L * (4 / (11 + 2*r));
        controller.notes = "Cohen-Coon PID tuning for FOPTD model.";

    case {"lambda", "lambda_tuning"}
        controller.name = 'Lambda Tuning';
        Lambda = 3 * L;
        controller.Kp = (Tau + L/2) / (K * (Lambda + L/2));
        Ti = Tau + L/2;
        Td = (Tau * L) / (2 * Tau + L);
        controller.notes = "Lambda tuning with lambda = 3L.";
        controller.Lambda = Lambda;

    case {"maximally_flat", "max_flat"}
        controller.name = 'Maximally Flat';
        controller.Kp = Tau / (K * L * exp(1));
        Ti = Tau;
        Td = L / 2;
        controller.notes = "Maximally Flat PID tuning.";

    case {"haalman"}
        controller.name = 'Haalman';
        controller.Kp = Tau / (2 * K * L);
        Ti = Tau;
        Td = L / 2;
        controller.notes = "Haalman PID tuning.";

    case {"amigo"}
        controller.name = 'AMIGO';
        controller.Kp = (1 / K) * (0.25 + 0.45 * (Tau / L));
        Ti = L * ((0.4 + 1.2 * (Tau / L)) / (1 + 0.05 * (Tau / L)));
        Td = L * (0.5 / (1 + 0.3 * (Tau / L)));
        controller.notes = "Astrom-Hagglund AMIGO PID tuning.";

    otherwise
        error('Unknown PID tuning method: %s', method);
end

controller.Ki = controller.Kp / Ti;
controller.Kd = controller.Kp * Td;
controller.Ti = Ti;
controller.Td = Td;
end
