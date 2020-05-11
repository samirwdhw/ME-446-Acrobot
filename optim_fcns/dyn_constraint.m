function [c,ceq] = dyn_constraint(sol,p,nVar, N,dt, xb, yb)
% This function specifies the dynamics as nonlinear constraints in 
% discrete time and the final constraint to ensure that the mass falls 
% in the bucket

params = p.params;
c = zeros(1);

ceq = zeros(4*N+1, 1);

%disp(x);

for i = 1:N
    
    % Collecting the states for system dynamics constraints from 
    % collocation point k to k+1
    x_k = sol(nVar*(i-1)+1:nVar*(i-1)+4);
    u_k = sol(nVar*(i-1)+5);
    
    x_kp1 = sol(nVar*i+1:nVar*i+4);
    u_kp1 = sol(nVar*i+5);

    q_k = x_k(1:2); 
    dq_k = x_k(3:4);
    q_kp1 = x_kp1(1:2);
    dq_kp1 = x_kp1(3:4);
    
    % Definining system dynamics at the collocation points
    De_k = fcn_De(q_k,params);
    Ce_k = fcn_Ce(q_k,dq_k,params);
    Ge_k = fcn_Ge(q_k,params);
    Be = [0;1];
    
    f_k = [dq_k; De_k\(Be*u_k - Ce_k*dq_k - Ge_k)];
    
    De_kp1 = fcn_De(q_kp1,params);
    Ce_kp1 = fcn_Ce(q_kp1,dq_kp1,params);
    Ge_kp1 = fcn_Ge(q_kp1,params);
    
    f_kp1 = [dq_kp1; De_kp1\(Be*u_kp1 - Ce_kp1*dq_kp1 - Ge_kp1)];
    
    % Writing the required nonlinear equality constraints
    ceq(4*(i-1)+1:4*i) = x_kp1 - x_k - dt/2*(f_k + f_kp1);
    
end

% Constraint to ensure ball falls into bucket
% Location of the ball in state space
qf = sol(end-4:end-3);
dqf = sol(end-2:end-1);

pos0 = fcn_p2(qf,params);
vel0 = fcn_J_foot(qf,params)*dqf;

x0 = pos0(1); y0 = pos0(2);
vx0 = vel0(1); vy0 = vel0(2);

te = (xb - x0)/vx0;

c = -vx0;

g = 9.81;

ceq(end) = yb - vy0*te + g*te^2/2 - y0;

%disp(x_k);
%disp(x_kp1);
%disp(u);

end

