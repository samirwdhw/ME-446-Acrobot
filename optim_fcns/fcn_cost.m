function val = fcn_cost(x,nVar,N,dt)
%This is the cost function for the optimization problem

val = 0;

u = x(5:5:end);

val = norm(u)^2*dt;


