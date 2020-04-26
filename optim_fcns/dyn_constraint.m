function [c,ceq] = dyn_constraint(x,p,nVar, N,dt)
%This function specifies the dynamics as nonlinear constraints in 
%discrete time

params = p.params;
c = [];

ceq = zeros(5*(N-1), 1);

for i = 1:N-1
    
    x_k = x(nVar*(i-1)+1:nVar*(i-1)+4);
    x_kp1 = x(nVar*i+1:nVar*i+4);
    u = x(nVar*(i-1)+5);

    q_k = x_k(1:2); 
    dq_k = x_k(3:4);
    q_kp1 = x_kp1(1:2);
    dq_kp1 = x_kp1(3:4);
    
    De = fcn_De(q_k,params);
    Ce = fcn_Ce(q_k,dq_k,params);
    Ge = fcn_Ge(q_k,params);
    Be = fcn_Be(q_k,params);

    dynamics = x_kp1 - x_k - dt*[dq_k;De\(Be*u - Ce*dq_k - Ge)];

    ceq(5*(i-1)+1) = dynamics(1);
    ceq(5*(i-1)+2) = dynamics(2);
    ceq(5*(i-1)+3) = dynamics(3);
    ceq(5*(i-1)+4) = dynamics(4);

end

end

