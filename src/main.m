%MAIN Entry point for the MIMO tank PID control project.
%
% This script compares classical PID tuning methods for the nonlinear
% stirred-tank level/temperature control problem and saves reproducible
% figures and metrics.

clear; clc; close all;

projectRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(fullfile(projectRoot, 'src')));

fprintf('Running MIMO tank PID controller comparison...\n');
resultsTable = compare_pid_methods();

fprintf('\nController comparison completed.\n');
disp(resultsTable);

fprintf('\nTo run the anti-windup study, execute:\n');
fprintf('  analyze_anti_windup()\n');
