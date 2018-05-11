function h = draw_drone( drone, color_body, alpha, axes_only )
%DRAW_DRONE(drone, alpha, axes_only) draws a drone cartoon

if isempty(drone)
    clear drone
    drone.prop_r = 0.07;
    drone.h = 0.023;
    drone.w = 0.1;
    drone.axis_l = 0.5;
end

% hold off
% axes
l = drone.axis_l;
ax(1) = patchline([0 l],[0 0],[0 0],...
    'EdgeColor','r','LineWidth',2,'EdgeAlpha',alpha);
ax(2) = patchline([0 0],[0 l],[0 0],...
    'EdgeColor','g','LineWidth',2,'EdgeAlpha',alpha);
ax(3) = patchline([0 0],[0 0],[0 l],...
    'EdgeColor','b','LineWidth',2,'EdgeAlpha',alpha);
if axes_only
    h = hgtransform('Parent',gca);
    set(ax,'Parent',h)
    return
end

x = drone.prop_r*cos(linspace(0,2*pi,50));
y = drone.prop_r*sin(linspace(0,2*pi,50));
prop_color = 'k';
color_edge_body = [19, 58, 4]/255;
% props
h_prop = drone.h;
alpha_prop = 0.3*alpha;
props(1) = fill3(x+drone.w,y+drone.w,-h_prop*ones(1,length(x)),...
    prop_color,'EdgeColor','none','FaceAlpha',alpha_prop);
hold on
props(2) = fill3(x+drone.w,y-drone.w,-h_prop*ones(1,length(x)),...
    prop_color,'EdgeColor','none','FaceAlpha',alpha_prop);
props(3) = fill3(x-drone.w,y+drone.w,-h_prop*ones(1,length(x)),...
    prop_color,'EdgeColor','none','FaceAlpha',alpha_prop);
props(4) = fill3(x-drone.w,y-drone.w,-h_prop*ones(1,length(x)),...
    prop_color,'EdgeColor','none','FaceAlpha',alpha_prop);
% body
cube = cube_plot([-0.5*drone.w -0.5*drone.w 0],...
    drone.w,drone.w,drone.h,color_body,color_edge_body, alpha);

cyl_transform = hgtransform('Parent',gca);
cyl = Cylinder([drone.w,drone.w,0],[drone.w,drone.w,-0.75*h_prop],...
    0.25*drone.prop_r,10,'k',1,0,alpha);
set(cyl,'Parent',cyl_transform)
arm = Cylinder([0 0 0.5*drone.h],[drone.w,drone.w,-0.4*h_prop],...
    0.007,8,color_body,1,0,alpha);
set(arm,'Parent',cyl_transform);
for rot = pi/2:pi/2:2*pi
    trans_cyl = hgtransform('Parent',cyl_transform);
    copyobj(cyl,trans_cyl);
    copyobj(arm,trans_cyl);
    set(trans_cyl(1),'Matrix',makehgtform('zrotate',rot));
end

h = hgtransform('Parent',gca);
set(props,'Parent',h)
set(cube,'Parent',h)
set(ax,'Parent',h)
set(cyl_transform,'Parent',h)

end

