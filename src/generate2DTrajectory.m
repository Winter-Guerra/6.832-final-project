function [trajectory] = generate2DTrajectory(r, v_nominal, dt)
% generate2DTrajectory Generates a (len(q), n) matrix. n is the number of
% samples of the trajectory. Does not generate the "ramp up" leg portion of
% the trajectory. That is assumed to be handled by a simple speed
% controller. len(q) is assumed to be 6.

% Length of trajectory
l = 2*pi*r;
t = l/v_nominal;

% Number of discrete samples
n = ceil(t/dt);

% Make some vectors
n_to_radian = 2*pi/n;
theta = (0:n-1)*n_to_radian;

% continuous loop equation
%x^2 + z^2 = radius^2;
x = r*sin(theta);
z = r - r*cos(theta);

% Velocities
dtheta = ones(1,n)*2*pi/t;
dx = v_nominal*cos(theta);
dz = v_nominal*sin(theta);

% Populate positions in trajectory
trajectory = [x; z; theta; dx; dz; dtheta];

% Do some analysis
g_max = (v_nominal^2/r)/9.81 + 1;

disp(["Trajectory takes ", t, " seconds. Max acceleration ", g_max, "g"]);


end

