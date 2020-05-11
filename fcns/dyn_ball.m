function x_dot = dyn_ball(t,x)
%This function simulates the dynamics of the ball at the end

g = 9.81;

x_dot = zeros(4,1);

x_dot(1) = x(3);
x_dot(2) = x(4);
x_dot(3) = 0;
x_dot(4) = -g;

end

