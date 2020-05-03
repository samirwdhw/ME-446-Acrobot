function u = fcn_controller(t, X, p, t_des,u_des, x_des)
% The function returns the control input required for the current time t
% using the ouputs of the optimization problem u_des and t_des

q = X(1:2);
dq = X(3:4);

%Finding the value higher and lower than the given time in t_des
closest_high = min(t_des(t_des>=t));
closest_low = max(t_des(t_des<=t));

% If the same value exists in t_des
if closest_high == closest_low
   u = u_des(t_des == t);
   X_des = x_des(t_des == t,:)';
% Using linear interpolation if the value does not exist in t_des
else
   alpha = (t - closest_low)/(closest_high - closest_low);
   
   u = (1-alpha)*u_des(t_des == closest_low) + alpha*u_des(t_des == closest_high);
   X_des = (1-alpha)*x_des(t_des == closest_low, :) + ...
           alpha*x_des(t_des == closest_high, :); 
   X_des = X_des';
end

% For LQR Control
Alin = fcn_Alin(q, dq, p, u);
Blin = fcn_Blin(q, dq, p);

%Cnt = [Blin, Alin*Blin, Alin^2*Blin, Alin^3*Blin];

%disp(q);

try 
    K = lqr(Alin, Blin, eye(4), eye(1));
catch
    K = zeros(1,4);
end

%disp(K);
%K = [-1 -1 -1 -1];

u = u - K*(X - X_des);

disp(X - X_des);

%disp(u);

end
