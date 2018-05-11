function [x] = getXVectorFromStateStruct(s)
%GETXVECTORFROMSTATESTRUCT 
% Computes 2D x vector from state struct (s)
n_x = 6;

% Turn rotation matrix into x,y,z angles
% Note: in the 2D case for the x_vector, we are defining theta as counter clockwise 
% (e.g. along an axis out of the page). This corresponds to theta_2d = -theta_y for the 3d case. 

euler = rotm2eul(s.rotation);
theta = -euler(2);
theta_dot = -angularVelocity(2);

x = [ s.position(1); s.position(3); theta; s.velocity(1); s.velocity(3); theta_dot ];

end

