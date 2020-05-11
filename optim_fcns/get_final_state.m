function xf = get_final_state(p, xb,yb)
% This function gives one valid final state that will lead to the ball
% falling into the bucket

g = 9.81;
params = p.params;
qf = [-pi/4; pi/6];
te = 1.5;

% Here x,v are final position and velocity in task space
pos0 = fcn_p2(qf, params);

x0 = pos0(1); y0 = pos0(2);

vx0 = (xb - x0)/te;
vy0 = (yb + g*te^2/2 - y0)/te;

J = fcn_J_foot(qf,params); J = J(1:2,1:2);

dqf = J\[vx0;vy0];

xf = [qf;dqf];

end

