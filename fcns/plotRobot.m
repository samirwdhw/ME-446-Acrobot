function plotRobot(X,p)

params = p.params;
ang = 0:0.1:2*pi;
L_boom = 1.1*params(3);

q = X(1:4);
p0 = [0 0 0]';
p1 = fcn_p1(q,params);
p2_aux = fcn_p2_aux(q,params);
p2 = fcn_p2(q,params);
p3 = fcn_p3(q,params);
p_toe = fcn_p_toe(q,params);

chain = [p0 p1 p2_aux p2 p3 p_toe];

plot3(chain(1,:),chain(2,:),chain(3,:),'k','LineWidth',3);
hold on
plot3(chain(1,:),chain(2,:),chain(3,:),'.r','LineWidth',4);
fill3(L_boom*cos(ang),L_boom*sin(ang),0*ang,0.5*[1 1 1])
axis equal
grid on
xlabel('X_0 [m]')
ylabel('Y_0 [m]')
zlabel('Z_0 [m]')

end
