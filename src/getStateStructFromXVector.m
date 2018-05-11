function [state] = getXVectorFromStateStruct(x)
%GETXVECTORFROMSTATESTRUCT 
% Computes state struct from x vector

q = x[1:3];
q_dot = x[4:6];


% Note: in the 2D case for the x_vector, we are defining theta as counter clockwise 
% (e.g. along an axis out of the page). This corresponds to theta_2d = -theta_y for the 3d case. 
state.position = [q(1); 0; q(2)];
state.velocity = [q_dot(1); 0; q_dot(2)];
state.rotation = eul2rotm([0.0, -q(3), 0]);
state.angularVelocity =   [0; -q_dot(3); 0];

end

