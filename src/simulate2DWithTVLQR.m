%% Clear the workspace
close all;
clear;
clc;

%% Add the plotting path to the current path to plot the drone
addpath('plotting');

%% Setup the simulation parameters
radius   = 1.0;
velocity = 1;
sim_dt   = 0.1;
simulationTime = 200 / sim_dt;

[constants] = getConstants();

T = [0];

%% Compute the projectile equations
% Highest point is z = 2r and x = 0
theta = 120 * pi / 180.0;
v0    = 6.0;
vx    = v0 * cos(theta);
vz    = v0 * sin(theta);

%% Simulate the loop
t = 0:0.01:0.25;
x_vec = [];
u_vec = [];
x = [-1 -1 0 0 0 0];
traj   = zeros(length(t), 6);
pitch  = zeros(length(t), 1);

% calculate nominal trajectory
[trajectory_nominal, u_f_matrix, t, s0] = generate2DTrajectory(0.01, constants);
constants.dt = t(2) - t(1);

% Define start and endpoints
start_p = [-2.5 0 0];
end_p = [0.2 -0.4 -2*pi];

start_x = [start_p s0 0 0];
end_x = [end_p 0 0 0];

[ramp_in, u_f_ramp_in, ramp_out, u_f_ramp_out] = generateRampingTrajectory(trajectory_nominal, constants);

% Prepend acceleration point and append stopping point
trajectory_nominal = [  ramp_in;
                        %[start_p s0 0 0]; 
                        trajectory_nominal; 
                        %[end_p s0 0 0];
                        ramp_out];
                                   
u_f_matrix = [  u_f_ramp_in;
                %[-constants.g*constants.m/2 -constants.g*constants.m/2];
                u_f_matrix;
                %[-constants.g*constants.m/2 -constants.g*constants.m/2]];
                u_f_ramp_out];

% Get TVLQR controller for nominal trajectory
[K_matrix] = getTVLQRMatrix(trajectory_nominal, u_f_matrix, constants);

% starting position.
%x = [0  0 0 s0 0 0 ];
x = trajectory_nominal(1,:);


% Hover controller.
% x_f = [1 1 0 0 0 0];
% Nominal thrust should counteract gravity
% u_f = [-constants.m*constants.g/2, -constants.m*constants.g/2];
% Position controller for a specific point.
% Replace this with a vector of K matrices for 
% applying TVLQR.
% K = lqrPositionController(x_f, u_f, constants);

k_idx_vector = [];

k_idx = 1;

for i = 2:20*length(t)
    % Wrap theta to [-2pi, 0] for continuous looping LQR.
    % Note, does some really bad things with start/stop trajectories
    %x_wrapped = [x(1:2) -wrapTo2Pi(-x(3)) x(4:6)];
    x_wrapped = x;
    
    % TVLQR Controller
    [K, u_f, x_f, k_idx] = getNearestKMatrix(x_wrapped, trajectory_nominal, K_matrix, u_f_matrix, k_idx);
    
    
    x_bar = x_wrapped - x_f;
    u = -K*x_bar' + u_f';
    
    u_vec = [u_vec; u'];
    k_idx_vector = [k_idx_vector k_idx];
    
    % Assure that actuation limits are followed
    u = max(u, 0);
    u = min(u, constants.tmax);
    
    % Add some noise to the control
    u = u; % + (0.5-rand([2 1])) * 0.1 * constants.tmax;
    
    % Simulate next step in dynamics
    [x,x_dot] = quadrotorDynamics2d(x, u, constants);
    
    % Populate vector of states for later visualization
    x_vec = [x_vec; x];    
    
    T = [T, T(end) + constants.dt];
end

% Do some statistical analysis
[RMSE, finalError, positionErrorTimeline] = analyzeTrajectory(x_vec, trajectory_nominal, constants);
disp('RMSE of trajectory is: ');
disp(RMSE);
disp('Final error in meters is: ');
disp(finalError);

figure(1); 
plot(positionErrorTimeline);

figure(2);
plot(k_idx_vector, 'x');

figure(3);
% Visualize(trajectory_nominal,1);
Visualize(x_vec, 3);

