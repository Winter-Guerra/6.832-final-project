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


%% Generate minimum velocities
min_speed_top = sqrt(g * radius)
min_omega_top = min_speed_top/radius;
min_KE_r_top = 0.5*J*min_omega_top^2;
min_KE_l_top = 0.5*constants.m*min_speed_top^2;
PE_top = constants.m * g*(2*radius);
min_E = min_KE_r_top + min_KE_l_top + PE_top;
min_speed_bottom = sqrt(2*min_E/(J/radius^2+constants.m));

%min_speed_bottom = sqrt(2*min_E/(constants.m));

s0 = min_speed_bottom;
% the energy is then
E = 0.5 * constants.m * s0^2;

z_raw = linspace(0,2 * radius ,1000);
speed_raw = sqrt(2*(E-constants.m * g * z_raw)/( J / radius^2+constants.m));
%speed_raw = sqrt(2*(E-constants.m * g * z_raw)/(constants.m));
omega_raw = speed_raw / radius;

t_raw = acos((radius - z_raw)/radius)./omega_raw;
time = linspace(0,t_raw(end),1000);
omega = interp1(t_raw,omega_raw,time);

time = [time,time(2:end)+time(end)];

omega = [omega,flip(omega(1:end-1))];
x = radius*sin(-omega.*time);
z = radius*(1-cos(omega.*time));

thrust = constants.m*radius*omega.^2;
alpha = diff(omega)./diff(time);
tau = J*alpha;

pitch = [0];
for i = 2:length(omega)
    pitch = [pitch, pitch(end) + omega(i) * (time(i) - time(i-1))];
end

state.velocity = [s0; 0; 0];
state.rotation = eul2rotm([0.0, 0.4, 0] * pi / 180.0)
state.angularVelocity = [0; 0; 0];
Energy = [E];
thrust_plot = [0];

%% Simulate the loop
thetas = [];
states = [state];

for i = 2:length(tau)
    
    constants.dt = time(i) - time(i-1);  
    a = state.angularVelocity(2)^2;
    inputs.thrust = -state.rotation*constants.g  + [0; 0; a * radius];
    inputs.thrust = [0; 0; inputs.thrust(3)];
    
    if (thrust(3) > constants.tmax)
        inputs.thrust = [0; 0; constants.tmax];
    end
    
    %inputs.thrust = [0; 0; thrust(i)];
    
    x = state.position(1);
    z = state.position(3);
   
   
    xc   = [0 0 radius];
    vec  = xc - [x  0 z];
    tvec = state.rotation * [0 0 1]';
    %tv   = [tvec(1) tvec(3)];
    
    %evec    = vec -tv;
    theta_e = -atan2(norm(cross(vec, tvec)), dot(vec,tvec)); 
    thetas  = [thetas, theta_e];
    
    a_error = state.angularVelocity(2) - theta_e/constants.dt;
   
    %dt2 = constants.dt ^2;
    inputs.torque = [0; 1.1 * theta_e; 0]; 
    %inputs.torque = [0; tau(i); 0];
    
    last_velocity = state.velocity;
    last_position = state.position;
    
    
    state = quadrotorDynamics(state, inputs, constants);
    %state.rotation = eul2rotm([0 -pitch(i) 0]);
    traj(i,1:3) = state.position';
    traj(i,4:6) = rotm2eul(state.rotation, 'zyx');
    states = [states;state];
    
    T = [T, T(end) + constants.dt];
    
%     disp (['sw: ', num2str(vecnorm(state.angularVelocity))])speed_raw = sqrt(2*(E-constants.m * g * z_raw)/( J / radius^2+constants.m));

%     disp (['sv: ', num2str(vecnorm(state.velocity))])
%     disp (['w: ', num2str(omega(i))])
%     disp (['v: ', num2str(speed_raw(i))])
    
    E = constants.m * g * state.position(3) ...
        + 0.5 * J * vecnorm(state.angularVelocity) ^2 + ...
        0.5 * constants.m * vecnorm(state.velocity) ^ 2
    Energy = [Energy; E];
    thrust_plot = [thrust_plot; inputs.thrust(3)];
end

figure(1); 
%plot(t(1,:), t(2,:), 'r');
hold on; plot(traj(:,1), traj(:,3),'gx');
pitch = atan2(traj(:,3), traj(:,1));
hold on; quiver(traj(1:5:end,1), traj(1:5:end,3), 0.01 * cos(pitch(1:5:end)), 0.01 * sin(pitch(1:5:end)) ); 
axis equal;

%figure(2);

Simulate(traj, states,2);

% 
% %%
% % Generate vertical circle
% t = generate2DTrajectory(radius, velocity, sim_dt);
% 
% traj = zeros(size(t,2), 6);
% requiredThrust = [];
% 
% % Simple propogation of the state
% i = 2;
% %while T(end) < simulationTime
% last_velocity = state.velocity;
% last_position = state.position;
% 
% kp_pos = 0.1;
% 
% load cmds
% 
% for i = 2:size(tau,2)
%     
%     % Compute simple required change in velocity between two states
% %     vx = (t(1,i) - t(1,i-1)) / constants.dt;
% %     vz = (t(2,i) - t(2,i-1)) / constants.dt;
% %     
% %     v  = ([t(1,i) 0 t(2,i)]' - last_position) / constants.dt;
% %     
% %     c = kp_pos * (v-last_velocity) / constants.dt;    
% %     inputs.thrust = state.rotation' * (-constants.g + c);
% %     requiredThrust = [requiredThrust, inputs.thrust];
% %     
% %     % Compute the required torque
% %     theta_next = atan2(t(2,i), t(1,i));
% %     theta_last = atan2(t(2,i-1), t(1,i-1));
% %     
% %     % Stop applying thrust after some time
% %     %if (i > 200)
% %     %    inputs.thrust = [0; 0; 0];
% %     %end
% % 
% %     inputs.thrust = [0; 0; inputs.thrust(3)];
% %     if (inputs.thrust(3) < 0)
% %         inputs.thrust = [0; 0; 0];
% %     end
% % 
% % %    inputs.thrust = [0; 0; inputs.thrust(3)];
% %     
% %     req_w = [0 (pi/2 - (theta_next - theta_last))/constants.dt 0]';
% %     req_torque = -(req_w - state.angularVelocity) / constants.dt;
% %     
% %     %inputs.torque = req_torque; 
% %     inputs.torque = [0; 0; 0];
%     
%     inputs.thrust = [0; 0; thrust(i)];
%     inputs.torque = [0; tau(i); 0];
%     
%     last_velocity = state.velocity;
%     last_position = state.position;
%     
%     constants.dt = t(i) - t(i-1);
%     
%     state = quadrotorDynamics(state, inputs, constants);
%     traj(i,1:3) = state.position';
%     traj(i,4:6) = rotm2eul(state.rotation, 'zyx');
%     
%     %disp (rotm2eul(state.rotation));
%     
%     distances = vecnorm(t(1:3) - state.position');
%     [M, idx]  = min(distances); 
%     
%     T = [T, T(end) + constants.dt];
% end
% 
% %figure(3);
% %plot (T(2:end),vecnorm(requiredThrust));
% 
% figure(1); 
% %plot(t(1,:), t(2,:), 'r');
% hold on; plot(traj(:,1), traj(:,3),'gx');
% pitch = atan2(traj(:,3), traj(:,1));
% hold on; quiver(traj(1:5:end,1), traj(1:5:end,3), 0.01 * cos(pitch(1:5:end)), 0.01 * sin(pitch(1:5:end)) ); 
% axis equal;
% 
% %figure(2);
% 
% Simulate(traj, 2);
