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
t = 0:0.01:3.75;
x_vec = [];
u_vec = [];
x = [0 0 0 0 0 0];
traj   = zeros(length(t), 6);
pitch  = zeros(length(t), 1);

% starting position.
x = [0  0 0 .5 0 .2 ];

pitch_vec = [x(3)];
states = [];


[trajectory, u_f_matrix, t] = generate2DTrajectory(0.01, constants);
x = trajectory(1,:);

for i = 2:length(t)    
    
    constants.dt = t(i) - t(i-1);
    xd = trajectory(i,:);
    u  = controlEnergy(x, xd, constants);
    
    u_vec = [u_vec; u];
    
    % Assure that actuation limits are followed
    u = max(u, 0);
    
    % Simulate next step in dynamics
    [x,x_dot] = quadrotorDynamics2d(x, u, constants);
    
    % Populate vector of states for later visualization
    x_vec = [x_vec; x];    
    
    T = [T, T(end) + constants.dt];
end

figure(1); 
plot(u_vec(1,:), u_vec(2,:));
%plot(t(1,:), t(2,:), 'r');
%hold on; plot(traj(:,1), traj(:,3),'gx');
%hold on; quiver(traj(1:5:end,1), traj(1:5:end,3), 0.01 * cos(pitch(1:5:end)), 0.01 * sin(pitch(1:5:end)) ); 

figure(2);

Visualize(x_vec, 2);