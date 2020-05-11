function u = fcn_controller(t, X, p, t_opt,u_opt, x_opt)
% The function returns the control input required for the current time t
% using the ouputs of the optimization problem u_des and t_des

u_des = u_interpol(t, t_opt, u_opt);
X_des = x_interpol(t, t_opt, x_opt, u_opt, p);

% For LQR Control
Alin = fcn_Alin(X_des(1:2), X_des(3:4), p.params, u_des);
Blin = fcn_Blin(X_des(1:2), X_des(3:4), p.params);

try 
    K = lqr(Alin, Blin, eye(4), eye(1));
catch
    K = zeros(1,4);
end

u = u_des - K*(X - X_des);

end
