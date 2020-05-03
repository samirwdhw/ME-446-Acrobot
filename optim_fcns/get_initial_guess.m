function sol0 = get_initial_guess(x0,xf,N)
% This function gives an initial guess for the optimization problem 
% as a linear interpolation between the initial and final states 

sol0= zeros(5*N,1);

sol0(1:4) = x0;
sol0(5) = 0; 

for i = 1:N

    sol0(5*i+1:5*i+4) = x0 + i/N*(xf-x0); 
    sol0(5*i+5) = 0;
end
    
end

