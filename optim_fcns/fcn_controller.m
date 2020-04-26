function u = fcn_controller(t,t_des,u_des)
% The function returns the control input required for the current time t
% using the ouputs of the optimization problem u_des and t_des


%Finding the value higher and lower than the given time in t_des
closest_high = min(t_des(t_des>=t));
closest_low = max(t_des(t_des<=t));

% If the same value exists in t_des
if closest_high == closest_low
   u = u_des(t_des == t);
% Using linear interpolation if the value does not exist in t_des
else
   alpha = (t - closest_low)/(closest_high - closest_low);
   u = (1-alpha)*u_des(t_des == closest_low) + alpha*u_des(t_des == closest_high); 
end

end
