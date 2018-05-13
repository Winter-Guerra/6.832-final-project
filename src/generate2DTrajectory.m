function [trajectory, u_f_matrix] = generate2DTrajectory(d_theta, constants)
% generate2DTrajectory Generates a (len(x), n) matrix. n is the number of
% samples of the trajectory. Does not generate the "ramp up" leg portion of
% the trajectory. That is assumed to be handled by a simple speed
% controller. len(q) is assumed to be 6.

% Min speed to get around loop
v_start = 7*sqrt(constants.radius);

% Starting energy
E_start = 0.5*constants.m*v_start^2;

% discretize theta
theta = linspace(pi/2, pi/2 - 2*pi, 2*pi/d_theta);

% positions
x_vec = constants.radius*cos(theta);
z_vec = constants.radius - constants.radius*sin(theta);
% Tangential velocity
velocity_tan = sqrt(2/constants.m * (E_start + constants.m*constants.g*z_vec));
% Velocity components
x_dot_vec = velocity_tan.*cos(theta);
z_dot_vec = velocity_tan*-1.*sin(theta);
d_theta = -velocity_tan/(2*pi*constants.radius);


% Populate positions in trajectory
trajectory = [x_vec', z_vec', theta'-pi/2, x_dot_vec', z_dot_vec', d_theta'];

% @TODO: Populate u_f vectors into matrix.
u_f_matrix = ones(length(theta), 2) .* [-constants.m*constants.g/2, -constants.m*constants.g/2];

% Do some analysis
% g_max = (v_nominal^2/r)/9.81 + 1;
% 
% disp(["Trajectory takes ", t, " seconds. Max acceleration ", g_max, "g"]);

% DEBUG
%figure(2);
%Visualize(trajectory,2);

end

