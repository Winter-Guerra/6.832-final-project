function Visualize(x_vec, fig, write_video)

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
%view(0,0);

h  = draw_drone([], 'b',1,0);
ht = hgtransform('Parent', gca); 

set(h, 'Parent', ht);
axis equal
ENU2NED = [0 1 0 0; 1 0 0 0; 0 0 -1 0; 0 0 0 1];
%axis([-3 3 -3 3 -3 3])

if (write_video)
    vidWriter = VideoWriter('flips.avi', 'Uncompressed AVI');
    open(vidWriter);
end

%while(true)
    for i = 1:num_t
        hold off;
        transformMatrix = eye(4);
        transformMatrix(1:3,1:3) = eul2rotm([0 x_vec(i,3) 0]);
        transformMatrix(1:3,4) = [x_vec(i,1) 0 x_vec(i,2)]';
     
        %transformMatrix(1:3,1:3) = eul2rotm([0 0 -x_vec(i,3)]);
        %transformMatrix(1:3,4) = [0 x_vec(i,1) -x_vec(i,2)]';
        
        set (ht, 'Matrix', transformMatrix);
        xlabel('x')
        ylabel('y')
        zlabel('z')
        pause(.01);
        grid on;
        axis([-5 4 -0.2 0.6 -0.8 2.0])
        
        if (write_video)
            myFrame = getframe(gca);
            writeVideo(vidWriter, myFrame);
        end
    end
%end

if (write_video)
    close(vidWriter);
end
end