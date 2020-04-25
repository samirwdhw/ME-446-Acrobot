% Acrobot
clc
clear all
close all

addpath gen
addpath fcns

fprintf('------ ME446 Milestone 5 -------\n')
fprintf('Initializing ..............\n')

% --- parameters ---
p = get_params;     % Getting physical parameters of the robot

% Initial condition
q0 = [0; 0]; %Joint angles
dq0 = [0; 0];       %Joint velocities
ic = [q0; dq0];

%Ploting the robot in the initial configuration:

% Recording
tstart = 0;
%tfinal = 2*Nstep;   %Maximum simulation time
tfinal = 5;
tout = tstart;
Xout = ic';

[tout,Xout] = ode45(@(t,X)dyn_manip(t,X,p),[tstart, tfinal], Xout(end,:));

fprintf('Simulation Complete!\n')

%% Visualing the motion
t = animateRobot(tout,Xout,p);


