function val = fcn_cost(x,nVar,N,dt)
%This is the cost function for the optimization problem

val = 0;

for i = 1:N
    
    val = val + x(nVar*(i-1)+5)^2*dt;
    
end


