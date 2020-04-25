function t = animateRobot(tin,Xin,p)

f = figure;
% v = VideoWriter('video.avi');
% open(v)
% set(f, 'doublebuffer', 'on');

N = p.N_animate;
[t,X] = even_sample(tin,Xin,N);

chain = zeros(3,3);

nt = length(t);
for i = 1:nt
    q = X(i,1:2);
    
    origin = [0;0;0];
    p1 = fcn_p1(q,p.params);
    p2 = fcn_p2(q,p.params);

    chain = [origin, p1, p2];
   
    plot(chain(1,:), chain(2,:),'k','LineWidth',3);
    title(['Time = ',num2str(t(i),'%.2f')]);
    xlabel('X (m)');
    ylabel('Y (m)');
    grid on; box on;
    xlim([-1 1]);
    ylim([-1 1]);

    drawnow
end

% close(v)




