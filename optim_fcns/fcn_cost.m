function val = fcn_cost(x,N,dt)
%This is the cost function for the optimization problem

u = x(5:5:end);

% Row vector to be multiplied to control inputs to get the cost
A = diag(ones(1,N+1));
A(1,1) = 0.5; A(end,end) = 0.5; 
A = dt*A;

val = u'*A*u;


