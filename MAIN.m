% Acrobot
clc
clear all
close all

addpath gen
addpath fcns
addpath optim_fcns

fprintf('------ ME446 Milestone 5 -------\n')
fprintf('Initializing ..............\n')

% --- parameters ---
p = get_params;     % Getting physical parameters of the robot

% Variables to study

% Initial condition
q0 = [-pi/2; 0]; %Joint angles
dq0 = [0; 0];       %Joint velocities
ic = [q0; dq0];

%Ploting the robot in the initial configuration:

% Recording
tstart = 0;
%tfinal = 2*Nstep;   %Maximum simulation time
tfinal = 2;
tout = tstart;
Xout = ic';

load('optim_vars.mat');

[tout,Xout] = ode45(@(t,X)dyn_manip(t,X,p,t_des,u_des, x_des),...
    [tstart, tfinal], Xout(end,:));

fprintf('Simulation Complete!\n')

% Back calculating u
u = backCalculate(tout, Xout, p, t_des, u_des, x_des);

%% Visualing the motion
t = animateRobot(tout,Xout,p);

%% Comparing Calculated and Actual Parameters

figure 
plot(t_des, u_des); hold on;
plot(tout, u);
title('Control Comparison');

figure 
plot(t_des, x_des(:,1)); hold on;
plot(tout, Xout(:,1));
title('Theta 1 Comparison');

figure
plot(t_des, x_des(:,2)); hold on;
plot(tout, Xout(:,2));
title('Theta 2 Comparison');


