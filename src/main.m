%% Clear the workspace
clear;
clc;

%% Add the plotting path to the current path to plot the drone
addpath('plotting');

%% Setup the simulation parameters
radius   = 5;
velocity = 1;
sim_dt   = 0.1;
constants.dt = sim_dt;
constants.g  = [0; 0; -9.81];
constants.J  = eye(3);
simulationTime = 200 / sim_dt;

% Initialize the state
state.position = [0; 0; 0];
state.velocity = [5; 0; 0];
state.rotation = eul2rotm([0.0, 0.1, 0]);
state.angularVelocity = [0; 0; 0];
T = [0];


% Generate vertical circle
t = generate2DTrajectory(radius, velocity, sim_dt);

traj = zeros(size(t,2), 6);
requiredThrust = [];

% Simple propogation of the state
i = 2;
%while T(end) < simulationTime
for i = 2:size(t,2)
    
    % Compute simple required change in velocity between two states
    vx = (t(1,i) - t(1,i-1)) / constants.dt;
    vz = (t(2,i) - t(2,i-1)) / constants.dt;
    
    c = ([vx; 0; vz]-state.velocity) / constants.dt;    
    inputs.thrust = -state.rotation * constants.g + c;
    requiredThrust = [requiredThrust, inputs.thrust];
    
    % Compute the required torque
    theta_next = atan2(t(2,i), t(1,i));
    theta_last = atan2(t(2,i-1), t(1,i-1));
    
    % Stop applying thrust after some time
    %if (i > 200)
    %    inputs.thrust = [0; 0; 0];
    %end
    
%     if (any(inputs.thrust < 0))
%         inputs.thrust = [0; 0; 0];
%     end

%    inputs.thrust = [0; 0; inputs.thrust(3)];
    
    req_w = [0 (theta_next - theta_last)/constants.dt 0]';
    req_torque = (req_w - state.angularVelocity) / constants.dt;
    
    inputs.torque = [0; 0; 0];
    
    state = quadrotorDynamics(state, inputs, constants);
    traj(i,1:3) = state.position';
    traj(i,4:6) = rotm2eul(state.rotation, 'zyx');
    
    distances = vecnorm(t(1:3) - state.position');
    [M, idx]  = min(distances); 
    
    T = [T, T(end) + constants.dt];
end

% figure(1);
% plot (T(2:end),vecnorm(requiredThrust));

figure(1); plot(t(1,:), t(2,:), 'r');
hold on; plot(traj(:,1), traj(:,3),'gx');
pitch = atan2(traj(:,3), traj(:,1));
hold on; quiver(traj(1:5:end,1), traj(1:5:end,3), 0.01 * cos(pitch(1:5:end)), 0.01 * sin(pitch(1:5:end)) ); 
axis equal;

figure(2);

Simulate(traj, 2);
