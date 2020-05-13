% Trajectory optimization for the acrobot

addpath fcns
addpath gen
addpath optim_fcns

clear

p = get_params;

% Initial State
x0 = [-pi/2;0;0;0];
u0 = 0;

% Bucket Location
xb = 2.0;
yb = -1;

% Final State for initial guess
xf = get_final_state(p,xb,yb);

% Decision Variables
nX = 4;
nU = 1;
N = 50;
tf = 2;
dt = tf/N;
t_opt = linspace(0,tf,N+1);

% Number of decision variables
nLamb = (nX + nU)*(N+1);
% Number of decision variables at each time step
nVar = nX+nU;

% Initial state constraints
Aeq1 = zeros(5,nLamb);
Aeq1(1:nVar,1:nVar) = eye(nVar);
beq1 = [x0;0];

Aeq = Aeq1;
beq = beq1;

% Motor Constraints are written as linear inequality constraints in terms
% of the decision variables
NH = p.params(24);
Tstall = p.params(29)*0.9;
w_NL = p.params(30);
Tlmax = p.params(31);

%Inequality constraints at one time step
A = [0, 0, 0,  0,                1/NH;
     0, 0, 0,  0,               -1/NH;
     0, 0, 0,  Tstall/w_NL*NH,   1/NH;
     0, 0, 0, -Tstall/w_NL*NH,  -1/NH];

% Creating a block diagonal repeating matrix out of this 
A = repmat(A, 1, N+1);
A = mat2cell(A, size(A,1), repmat(nVar,1,N+1));
A = blkdiag(A{:});

b = [Tlmax; Tlmax; Tstall; Tstall];
b = repmat(b, N+1,1);

%% Optimization

sol0 = get_initial_guess(x0,xf,N);
%load('prev_sol');
%sol0 = sol;

options = optimoptions('fmincon',...
'Display','iter','Algorithm','interior-point', ...
'MaxIter', 10000, 'MaxFunEvals', 100000,'ConstraintTolerance',1e-6, ...
'OptimalityTolerance',1e-2);

fprintf('Starting Optimization to generate a good initial guess........\n');

sol0 = fmincon(@(x)fcn_cost(x,N,dt), sol0, [], [],Aeq,beq,...
[],[],@(x)dyn_constraint(x,p,nVar, N,dt, xb, yb),options);

fprintf('Starting Optimization with actuator dynamics........... \n');

sol = fmincon(@(x)fcn_cost(x,N,dt), sol0, A, b,Aeq,beq,...
[],[],@(x)dyn_constraint(x,p,nVar, N,dt, xb, yb),options);

[x_opt,u_opt] = get_optimal_vals(sol, N, nVar);

%% Simulation for the mass at end

xf = x_opt(end,:)';
qf = xf(1:2);
dqf = xf(3:4);
J = fcn_J_foot(qf,p.params); J = J(1:2,1:2);

ball_pos0 = fcn_p2(qf,p.params); ball_pos0 = ball_pos0(1:2);
ball_v0 = J*dqf;

ball_x0 = [ball_pos0;ball_v0];

[ball_t, ball_X] = ode45(@(t,X)dyn_ball(t,X),[t_opt(end) t_opt(end)+10], ball_x0);

%% Plotting errors in this solution

t_sample = linspace(0,tf,(N+1)*100);

error = get_estimation_error(t_sample, t_opt, u_opt, x_opt, p);

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
animateRobotOptim(t_opt, x_opt, ball_t, ball_X, xb, yb, p);

%%
save('optim_vars', 'x_opt', 'u_opt', 't_opt', 'xb', 'yb');
%save('prev_sol', 'sol');