% Trajectory optimization for the acrobot

addpath fcns
addpath gen
addpath optim_fcns

clear

p = get_params;

% Initial State
x0 = [-pi/2;0;0;0];
u0 = 0;
% Decision Variables
nX = 4;
nU = 1;
N = 250;
tf = 5;
dt = tf/N;
t_des = linspace(0,tf,N);

% Number of decision variables
nLamb = (nX + nU)*N;
nVar = nX+nU;

% Initial state constraints
Aeq1 = zeros(4,nLamb);
Aeq1(1:nX+nU,1:nX+nU) = eye(nX+nU);
beq1 = [x0;0];

% Final state constraints
Aeq2 = zeros(4,nLamb);
Aeq2(1,(nX+nU)*(N-1)+1) = 1;
Aeq2(2,(nX+nU)*(N-1)+2) = 1;
Aeq2(3,(nX+nU)*(N-1)+3) = 1;
Aeq2(4,(nX+nU)*(N-1)+4) = 1;
beq2 = [pi/2;0;0;0];

Aeq = [Aeq1; Aeq2(1:2,:)];
beq = [beq1; beq2(1:2)];

sol0 = zeros(nLamb,1);

options = optimoptions('fmincon',...
'Display','iter','Algorithm','interior-point', ...
'MaxIter', 10000, 'MaxFunEvals', 100000,'ConstraintTolerance',1e-4, ...
'OptimalityTolerance',1e-2);

sol = fmincon(@(x)fcn_cost(x,nVar,N,dt), sol0, [],[],Aeq,beq,...
[],[],@(x)dyn_constraint(x,p,nVar, N,dt),options);

[x_des,u_des] = get_optimal_vals(sol, N, nVar);

%% 
animateRobot(t_des, x_des, p);

%%
save('optim_vars', 'x_des', 'u_des', 't_des');