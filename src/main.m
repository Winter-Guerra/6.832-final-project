clear;
clc;

addpath('plotting');
addpath('../src');

radius   = 5;
velocity = 1;
sim_dt   = 0.1;

% Simulate vertical circle
t = generate2DTrajectory(radius, velocity, sim_dt);

% Simple state

state = zeros(size(t,2), 6);
state(:,1) = t(1,:)';
state(:,3) = t(2,:)';
% figure(1); plot(t(1,:), t(2,:), 'r');
% hold on; plot(state(:,1), state(:,3),'g');

figure(1);
fig = gcf;

Simulate(state, fig);