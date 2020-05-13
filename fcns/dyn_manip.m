function [dXdt, u] = dyn_manip(t,X,p, t_des, u_des, x_des)

params = p.params;

% To seperate the states of robot and power consumed
X = X(1:4);

q = X(1:2);
dq = X(3:4);

De = fcn_De(q,params);
Ce = fcn_Ce(q,dq,params);
Ge = fcn_Ge(q,params);
Be = fcn_Be(q,params);

% Design your controller here
u = fcn_controller(t, X, p, t_des,u_des, x_des);

ddq = De \ (Be * u - Ce * dq - Ge);
% ddq = De \ (Be * u + J' * F_ext - Ce*dq - Ge);

% Calculation of power

% Motor parameters
NH = params(24);
Kv = params(25);
Kt = params(26);
Rw = params(27);

IH = u/NH/Kt;
VH = Rw*IH + Kv*dq(2)*NH;

power = VH*IH;

dXdt = [dq;ddq; power];
