function t = animateRobotOptim(tin,Xin, ball_t, ball_X, xb, yb, p)

f = figure;
% v = VideoWriter('video.avi');
% open(v)
% set(f, 'doublebuffer', 'on');

N = p.N_animate;

%[t,X] = even_sample(tin,Xin,N);
t = tin; X = Xin;

nt = length(t);

pgon = polyshape([xb-0.15 xb+0.15 xb+0.1 xb-0.1], [yb yb yb-.3 yb-.3]);

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
    % Drawing Ball
    rectangle('Position', pos, 'Curvature', [1 1],'FaceColor','r');
    % Drawing Bucket
    plot(pgon);
    
    title(['Time = ',num2str(t(i),'%.2f')]);
    xlabel('X (m)');
    ylabel('Y (m)');
    grid on; box on;
    axis equal;
    xlim([-1 3]);
    ylim([-2 1]);

    drawnow
end

[ball_teven,ball_Xeven] = even_sample(ball_t,ball_X, N);

for i = 1:length(ball_teven)
   
    clf;
    
    pos = ball_Xeven(i,:); pos = [pos(1:2)-[0.2 0.2]/2, 0.2, 0.2];
    
    plot(chain(1,:), chain(2,:),'k','LineWidth',3); hold on;
    rectangle('Position', pos, 'Curvature', [1 1],'FaceColor','r');
    plot(pgon);
    title(['Time = ',num2str(ball_teven(i),'%.2f')]);
    xlabel('X (m)');
    ylabel('Y (m)');
    grid on; box on;
    axis equal;
    xlim([-1 3]);
    ylim([-2 1]);
    
    
    drawnow
    
    if pos(1)-0.1 >= xb
        break
    end
end

rectangle('Position', pos, 'Curvature', [1 1],'FaceColor','r'); hold on;
plot(pgon);
drawnow

% close(v)




