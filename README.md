# MIMO Tank Level and Temperature Control Using Classical PID Tuning Methods

A MATLAB-based control systems project for modeling, linearizing, and controlling a nonlinear stirred-tank process with coupled level and temperature dynamics.

## Overview

This project studies a two-input two-output industrial tank system. The controlled outputs are liquid level and tank temperature. The manipulated inputs are inlet flow rate and heater power.

The project includes nonlinear process modeling, linearization around an operating point, feedforward decoupling, PI/PID controller design, nonlinear simulation, and comparison of classical PID tuning methods.

## Main Features

- Nonlinear MIMO tank model based on mass and energy balance
- Linearized transfer-function representation
- Feedforward decoupling for loop interaction reduction
- Classical PID tuning methods: Ziegler-Nichols, CHR, Cohen-Coon, Lambda, Maximally Flat, Haalman, and AMIGO
- Nonlinear closed-loop simulation using `ode45`
- Automatic export of comparison figures and metrics

## Repository Structure

```text
mimo-tank-pid-control/
├── README.md
├── .gitignore
├── requirements.md
├── src/
│   ├── main.m
│   ├── compare_pid_methods.m
│   ├── analyze_anti_windup.m
│   ├── config/
│   ├── controllers/
│   ├── models/
│   ├── simulation/
│   ├── visualization/
│   └── legacy/
├── figures/
├── results/
└── docs/
```

## Usage

Open MATLAB in the repository root and run:

```matlab
run("src/main.m")
```

This runs the controller comparison and saves:

- `figures/controller_temperature_comparison.png`
- `figures/setpoint_tracking_zoom.png`
- `figures/disturbance_rejection_zoom.png`
- `figures/heater_power_comparison_kw.png`
- `results/controller_comparison_metrics.csv`

To run the Lambda anti-windup comparison:

```matlab
analyze_anti_windup()
```

## Notes

The `src/legacy/` folder contains the original project scripts. The cleaned implementation uses modular functions and parameter structures instead of duplicated script logic and global variables.

| Controller | Peak Temp. (°C) | Overshoot (°C) | Disturbance Min. (°C) | Disturbance Dip (°C) | Settling Time (s) | Final Temp. (°C) |
|---|---:|---:|---:|---:|---:|---:|
| Ziegler–Nichols PID | 51.634 | 1.634 | 49.972 | 0.028 | 154.24 | 50.000 |
| CHR 0% Overshoot | 50.109 | 0.109 | 49.880 | 0.120 | 2312.00 | 49.994 |
| Cohen–Coon | 51.327 | 1.327 | 49.965 | 0.035 | 181.57 | 50.000 |
| Lambda Tuning | 50.158 | 0.158 | 49.716 | 0.284 | 3118.50 | 49.984 |
| Maximally Flat | 50.142 | 0.142 | 49.781 | 0.219 | 2858.10 | 49.991 |
| Haalman | 50.119 | 0.119 | 49.847 | 0.153 | 2522.10 | 49.993 |
| AMIGO | 50.242 | 0.242 | 49.789 | 0.211 | 2398.30 | 50.000 |

## Results and Visualizations

### Full Controller Comparison

![Controller temperature comparison](figures/controller_temperature_comparison.png)

### Setpoint Tracking Detail

![Setpoint tracking zoom](figures/setpoint_tracking_zoom.png)

### Disturbance Rejection Detail

![Disturbance rejection zoom](figures/disturbance_rejection_zoom.png)

### Heater Power Control Action

![Heater power comparison](figures/heater_power_comparison_kw.png)