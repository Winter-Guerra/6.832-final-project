function Simulate(state, fig)

figure(fig);
holdFig = ishandle(fig);
xlabel('x')
ylabel('y')
zlabel('z')
%if ~holdFig
%     axis equal
%     grid on
%     view([-111,-31])
%     axis([-5 5 -5 5 -4 -1]);
%     camroll(180)
% end

axis equal;
grid on;
axis([-10 10 -10 10 -10 10]);
plot3 (state(:,1), state(:,2), state(:,3))

h  = draw_drone([], 'b',1,0);
ht = hgtransform('Parent', gca); 
set(h, 'Parent', ht);

for i = 1:10:size(state,1)
    hold off;
    txy = makehgtform('translate', state(i,1:3),'zrotate', state(i,4), 'yrotate', state(i,5), 'xrotate', state(i,6));
    set (ht, 'Matrix', txy);
    pause(0.1);
end
end