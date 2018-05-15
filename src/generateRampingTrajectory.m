function [ramp_in, u_f_ramp_in, ramp_out, u_f_ramp_out] = generateRampingTrajectory(trajectory_nominal, constants)
%GENERATERAMPINGTRAJECTORY 

% Ramp into the trajectory
vel_in_x = trajectory_nominal(1,4);
vel_in_y = trajectory_nominal(1,5);

% Rammp up the velocity
ramp_time  = 1.5;
t_ramp     = linspace(0,ramp_time,ramp_time/constants.dt);
constant_a = (trajectory_nominal(1,4))/ramp_time;

% Calculate nominal thrust required
f_x = constant_a*constants.m;
f_z = -constants.m*constants.g;
total_thrust = vecnorm([f_x f_z]);
u_f_ramp_in = repmat([total_thrust/2 total_thrust/2], length(t_ramp), 1);     
u_f_ramp_out = u_f_ramp_in;


% Calculate velocities and positions
vel_x_ramp = constant_a.*t_ramp;
%vel_x_ramp = linspace(0,vel_in_x,length(t_ramp));
x_ramp = cumtrapz(t_ramp, vel_x_ramp);
x_ramp = -flip(x_ramp);
z_ramp = zeros(size(x_ramp));
zeros_ramp = zeros(size(x_ramp));
pitch_ramp = -atan2(-f_x,f_z).* ones(size(x_ramp))*.5;
%pitch_ramp = 0.2.* ones(size(x_ramp));


ramp_in = [x_ramp' z_ramp' pitch_ramp' vel_x_ramp' zeros_ramp' zeros_ramp'];

ramp_out = [-flip(x_ramp)' z_ramp' -pitch_ramp'-2*pi flip(vel_x_ramp)' zeros_ramp' zeros_ramp'];

% Make sure that the last point in the deceleration has the correct u_f for
% level hover.
ramp_out(end, 3) = -2*pi;
u_f_ramp_out(end, :) = [-constants.m*constants.g/2 -constants.m*constants.g/2];



end

