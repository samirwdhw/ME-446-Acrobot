function error = get_estimation_error(t_sample, t_des, u_des, x_des,p)
% This function gives the error in the differential equation at the 
% sample points in t_sample

error = zeros(4,length(t_sample));

params = p.params;

for i = 1:length(t_sample)
    t = t_sample(i);
    closest_low = max(t_des(t_des<=t));
    
    u = u_interpol(t,t_des,u_des);
    x = x_interpol(t,t_des,x_des, u_des, p);
    
    x_k = x_des(t_des == closest_low,:)';
    u_k = u_des(t_des == closest_low);
    
    dt = t - closest_low;
    
    q_k = x_k(1:2); 
    dq_k = x_k(3:4);
    q = x(1:2);
    dq = x(3:4);
    
    % Definining system dynamics at the collocation points
    De_k = fcn_De(q_k,params);
    Ce_k = fcn_Ce(q_k,dq_k,params);
    Ge_k = fcn_Ge(q_k,params);
    Be = [0;1];
    
    f_k = [dq_k; De_k\(Be*u_k - Ce_k*dq_k - Ge_k)];
    
    De = fcn_De(q,params);
    Ce = fcn_Ce(q,dq,params);
    Ge = fcn_Ge(q,params);
    
    f_kp1 = [dq; De\(Be*u - Ce*dq - Ge)];
    
    error(:,i) = x - x_k - dt/2*(f_k + f_kp1);
    
end

end

