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
constants.g  = -g;
constants.J  = J;
simulationTime = 200 / sim_dt;
constants.tmax = 3*g;

% Initialize the state
state.position = [0; 0; 0];
T = [0];

%% Compute the projectile equations
% Highest point is z = 2r and x = 0
theta = 120 * pi / 180.0;
v0    = 6.0;
vx    = v0 * cos(theta);
vz    = v0 * sin(theta);

%% Simulate the loop
thetas = [];
state.rotation = eul2rotm([0 0 0]);
x = [1 0 theta vx vz 0];
t = 0:0.01:0.5;
traj   = zeros(length(t), 6);
pitch  = zeros(length(t), 1);
states = [];

for i = 2:length(t)
    constants.dt = 0.01;  
    inputs.thrust = 0;
    inputs.torque = 0;
    
    x = quadrotorDynamics2d(x, inputs, constants);
    
    state.position = [x(1) 0 x(2)];
    state.velocity = [x(4) 0 x(5)];
    state.angularVelocity = [0 0 0];

    % Use a zero centered circle for simplicity
    radius_vec  = [x(1) x(2)];
    tangent_vec = null(radius_vec(:)');
    pitch = -atan2(tangent_vec(2), tangent_vec(1));
    state.rotation = eul2rotm([0 pitch 0]);
    
    traj(i,1:3) = state.position';
    traj(i,4:6) = rotm2eul(state.rotation, 'zyx');
%     disp(state.velocity');
    pitch(i) = traj(i,5);
    states = [states; state];
    T = [T, T(end) + constants.dt];
end

figure(1); 
%plot(t(1,:), t(2,:), 'r');
%hold on; plot(traj(:,1), traj(:,3),'gx');
%hold on; quiver(traj(1:5:end,1), traj(1:5:end,3), 0.01 * cos(pitch(1:5:end)), 0.01 * sin(pitch(1:5:end)) ); 
axis equal;

figure(2);

Simulate(traj,states, 2);