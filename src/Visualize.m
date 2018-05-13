function Visualize(x_vec, fig)

% X_vec is size (t, len(x)). E.g. one row for each t_i.
[num_t, len_x] = size(x_vec); 

figure(fig);
holdFig = ishandle(fig);
xlabel('x')
ylabel('y')
zlabel('z')

axis equal;
grid on;
plot3 (x_vec(:,1), zeros(num_t, 1), x_vec(:,2));
hold on;
view(0,0);

h  = draw_drone([], 'b',1,0);
ht = hgtransform('Parent', gca); 
set(h, 'Parent', ht);

%while(true)
    for i = 1:num_t
        hold off;
        transformMatrix = eye(4);
        transformMatrix(1:3,1:3) = eul2rotm([0 x_vec(i,3) 0]);
        transformMatrix(1:3,4) = [x_vec(i,1) 0 x_vec(i,2)]'; 
        set (ht, 'Matrix', transformMatrix);
        pause(.1);
    end
%end    
end