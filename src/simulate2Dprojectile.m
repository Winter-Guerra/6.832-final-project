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
constants.m  = 1;
% constants.dt = sim_dt;
g = 9.8;
%g=0.1;
J = 5e-2;
simulationTime = 200 / sim_dt;

% Set all constants
constants.g  = -g;
constants.J  = J;
constants.tmax = 3*g;
constants.dt = 0.01;
constants.radius = radius;
% Define the width of the quadrotor (distance between motors)
constants.baseline = 0.1;

T = [0];

%% Compute the projectile equations
% Highest point is z = 2r and x = 0
theta = 120 * pi / 180.0;
v0    = 6.0;
vx    = v0 * cos(theta);
vz    = v0 * sin(theta);

%% Simulate the loop
x_vec = [];
x = [1 0 theta vx vz 0];
t = 0:0.01:0.5;
traj   = zeros(length(t), 6);
pitch  = zeros(length(t), 1);

x = [0 0 0 0 0 0];
[u,t, x] = controllerEnergy(x, [], constants);
pitch_vec = [x(3)];
states = [];


for i = 2:length(t)    
    % Calculate input from controller
    %u = [50.0 0]; % F_1, F_2
    constants.dt = t(i) - t(i-1);
    uin = u(i,:);
    
    % Simulate next step in dynamics
    x = quadrotorDynamics2d(x, uin, constants);
    
    % Populate vector of states for later visualization
    x_vec = [x_vec; x];    
    
    T = [T, T(end) + constants.dt];
end

figure(1); 
plot(x_vec(:,3));
%plot(t(1,:), t(2,:), 'r');
%hold on; plot(traj(:,1), traj(:,3),'gx');
%hold on; quiver(traj(1:5:end,1), traj(1:5:end,3), 0.01 * cos(pitch(1:5:end)), 0.01 * sin(pitch(1:5:end)) ); 

figure(2);

Visualize(x_vec, 2);