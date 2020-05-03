function [x,u] = get_optimal_vals(sol,N, nVar)
%This function converts the solution of the optimization problem
%into states and the control input

x = zeros(4, N+1);
u = zeros(1, N+1);

u = sol(5:nVar:end);

for i = 1:4
    
    x(i,:) = sol(i:nVar:end);

end

x = x';

end

