function Simulate(state, fig)

holdFig = ishandle(fig);
%figure(fig);
xlabel('x')
ylabel('y')
zlabel('z')
if ~holdFig
    axis equal
    grid on
    view([-111,-31])
    axis([-5 5 -5 5 -4 -1]);
    camroll(180)
end

axis equal;
grid on;
%axis([-10 10 -10 10 0 7]);
plot3 (state(:,1), state(:,2), state(:,3))

h  = draw_drone([], 'b', 1,0);
ht = hgtransform('Parent', gca); 
set(h, 'Parent', ht);

for i = 1:size(state,1)
    hold off;
    txy = makehgtform('translate', state(i,1:3));
    rotz = makehgtform('zrotate', 0.0);
    roty = makehgtform('yrotate', 0.0);
    rotx = makehgtform('xrotate', 0.0);
    set (ht, 'Matrix', txy);
    pause(0.01);
end
end