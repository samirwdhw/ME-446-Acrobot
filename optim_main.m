% Trajectory optimization for the acrobot

addpath fcns
addpath gen
addpath optim_fcns

clear

p = get_params;

% Initial State
x0 = [-pi/2;0;0;0];
u0 = 0;

% Final State
xf = [pi/2;0;0;0];

% Decision Variables
nX = 4;
nU = 1;
N = 50;
tf = 2;
dt = tf/N;
t_des = linspace(0,tf,N);

% Number of decision variables
nLamb = (nX + nU)*(N+1);
% Number of decision variables at each time step
nVar = nX+nU;

% Initial state constraints
Aeq1 = zeros(5,nLamb);
Aeq1(1:nVar,1:nVar) = eye(nVar);
beq1 = [x0;0];

% Final state constraints
Aeq2 = zeros(4,nLamb);
Aeq2(1:4,nLamb-4:nLamb-1) = eye(4);
beq2 = xf;

Aeq = [Aeq1; Aeq2];
beq = [beq1; beq2];

%% Optimization

sol0 = get_initial_guess(x0,xf,N);

options = optimoptions('fmincon',...
'Display','iter','Algorithm','interior-point', ...
'MaxIter', 10000, 'MaxFunEvals', 100000,'ConstraintTolerance',1e-4, ...
'OptimalityTolerance',1e-3);

sol = fmincon(@(x)fcn_cost(x,N,dt), sol0, [],[],Aeq,beq,...
[],[],@(x)dyn_constraint(x,p,nVar, N,dt),options);

[x_des,u_des] = get_optimal_vals(sol, N, nVar);

%% Plotting errors in this solution

t_sample = linspace(0,tf,N*100);

error = get_estimation_error(t_sample, t_des, u_des, x_des, p);

figure
subplot(2,2,1);
plot(t_sample, error(1,:));
title('\theta_1 dynamics error');
xlabel('Time (s)');
ylabel('Error');
subplot(2,2,2);
plot(t_sample, error(2,:));
title('\theta_2 dynamics error');
xlabel('Time (s)');
ylabel('Error');
subplot(2,2,3);
plot(t_sample, error(3,:));
title('\omega_1 dynamics error');
xlabel('Time (s)');
ylabel('Error');
subplot(2,2,4);
plot(t_sample, error(4,:));
title('\omega_1 dynamics error');
xlabel('Time (s)');
ylabel('Error');

%% Animation of this system 
animateRobot(t_des, x_des, p);

%%
save('optim_vars', 'x_des', 'u_des', 't_des');