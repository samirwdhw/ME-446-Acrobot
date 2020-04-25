function dXdt = dyn_manip(t,X,p)

params = p.params;

q = X(1:2);
dq = X(3:4);

De = fcn_De(q,params);
Ce = fcn_Ce(q,dq,params);
Ge = fcn_Ge(q,params);
Be = fcn_Be(q,params);

% Design your controller here
u = 0;

ddq = De \ (Be * u - Ce * dq - Ge);
% ddq = De \ (Be * u + J' * F_ext - Ce*dq - Ge);

dXdt = [dq;ddq];
