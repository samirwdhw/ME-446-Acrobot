function t = animateRobot(tin,Xin, p)

f = figure;
% v = VideoWriter('video.avi');
% open(v)
% set(f, 'doublebuffer', 'on');

N = p.N_animate;

[t,X] = even_sample(tin,Xin,N);
%t = tin; X = Xin;

nt = length(t);

for i = 1:nt
    
    clf
    
    q = X(i,1:2)';
    
    origin = [0;0;0];
    p1 = fcn_p1(q,p.params);
    p2 = fcn_p2(q,p.params);
    
    pos = [p2(1:2)'-[0.2 0.2]/2, 0.2, 0.2];
    
    chain = [origin, p1, p2];
    
    % Drawing Bot
    plot(chain(1,:), chain(2,:),'k','LineWidth',3); hold on;
    
    title(['Time = ',num2str(t(i),'%.2f')]);
    xlabel('X (m)');
    ylabel('Y (m)');
    grid on; box on;
    axis equal;
    xlim([-1 3]);
    ylim([-2 1]);

    drawnow
end

% close(v)




