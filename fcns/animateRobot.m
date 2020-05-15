function t = animateRobot(tin,Xin, ball_t, ball_X, xb, yb, p, t_opt)

f = figure;
% v = VideoWriter('video.avi');
% open(v)
% set(f, 'doublebuffer', 'on');

% Resetting indices of ball_X to match times with tin
index_throw = find(tin == t_opt(end));
ball_t = [tin(1:index_throw-1); ball_t];
ball_X = [zeros(index_throw-1,4); ball_X];

ball_radius = 0.1*[2 2];

t = tin; X = Xin;

nt = length(t);

pgon = polyshape([xb-0.15 xb+0.15 xb+0.1 xb-0.1], [yb yb yb-.3 yb-.3]);

for i = 1:nt
    
    clf
    
    q = X(i,1:2)';
    
    origin = [0;0;0];
    p1 = fcn_p1(q,p.params);
    p2 = fcn_p2(q,p.params);
    
    chain = [origin, p1, p2];
    
    % Drawing Bot
    plot(chain(1,:), chain(2,:),'k','LineWidth',3); hold on;
    % Drawing Ball
    if t(i) <= t_opt(end)
        % Place ball at end of manipulator
        pos = [p2(1:2)'-ball_radius/2, ball_radius];
    else
        % Ball projectile motion
        pos = ball_X(i,:); pos = [pos(1:2)-ball_radius/2, ball_radius];
    end
    
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
    
    % Deciding end of animation
    if pos(1) >= xb - 0.1
        break
    end
end

rectangle('Position', pos, 'Curvature', [1 1],'FaceColor','r'); hold on;
plot(pgon);
drawnow

% close(v)

