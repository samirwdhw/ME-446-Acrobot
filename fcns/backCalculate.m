function u = backCalculate(tout, Xout, p, t_opt, u_opt, x_opt)
% This function returns the control input used in the simulation

iters = length(tout);

u = zeros(iters,1);

for i = 1:iters
   
    t = tout(i);
    X = Xout(i,:)';
    
    u(i) = fcn_controller(t, X, p, t_opt,u_opt, x_opt);
    
end



end

