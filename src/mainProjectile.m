%% Clear the workspace
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

constants.g  = [0; 0; -g];
constants.J  = J * eye(3);
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

state.position = [0.0; 0; 0];
state.velocity = [vx; 0; vz];
state.rotation = eul2rotm([0 30 0]);
state.angularVelocity = [0; 0; 0];

TT = 2 * v0 * sin(theta) / g;

%% Hit target velocity
v_x = linspace(0, vx, 200);
v_z = linspace(0, vz, 200);
vel = linspace(0, v0*5, 200);

t = 0:0.01:TT; %2+TT;

theta = 0:2*pi;

state.position = [1.0; 0; 0];
state.velocity = [vx; 0; vz];
state.rotation = eul2rotm([0 -30 0] * pi / 180);
state.angularVelocity = [0; 0; 0];

thrust = [0 0 0];
%thrust = [thrust; zeros(199,1), zeros(199,1), diff(vel)'/0.01];
thrust = [thrust; diff(v_x)'/0.01 zeros(199,1) ones(199,1)*g/5]; %+diff(v_z)'/0.01];
thrust = [thrust; zeros(floor(TT/0.01), 3)];


%% Simulate the loop
thetas = [];
traj   = zeros(length(t), 6);
pitch  = zeros(length(t), 1);
states = [];

for i = 2:length(t)-10
    constants.dt = 0.01;  
    inputs.thrust = thrust(i,:)'; %state.rotation' * thrust(i,:)'; %[0; 0; 0];
    inputs.torque = [0; 0; 0];
    
    state = quadrotorDynamics(state, inputs, constants);
    
    r = sqrt(state.position(1)^2 + state.position(3)^2)
    p = -pi/2 - asin(state.position(3)/ r);
    p*180/pi
    state.rotation = eul2rotm([0 -p 0]);
    
    
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

