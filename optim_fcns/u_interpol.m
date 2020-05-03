function u = u_interpol(t, t_des, u_des)
% Given the control input at discrete times, this function gives the  
% control input at any point t using piecewise linear interpolation
% between the discrete points

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

