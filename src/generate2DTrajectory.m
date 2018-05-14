function [trajectory, u_f_matrix, t] = generate2DTrajectory(d_theta, constants)
% generate2DTrajectory Generates a (len(q), n) matrix. n is the number of
% samples of the trajectory. Does not generate the "ramp up" leg portion of
% the trajectory. That is assumed to be handled by a simple speed
% controller. len(q) is assumed to be 6.
% Also returns the expected nominal thrust at every point in trajectory. 
% u_f_matrix (num_fixed_points, len(u))

min_speed_top = sqrt(-constants.g * constants.radius);
min_omega_top = min_speed_top/constants.radius;
min_KE_l_top = 0.5* constants.m *min_speed_top^2;
PE_top = -constants.m*constants.g*(2*constants.radius);
min_E = min_KE_l_top + PE_top;
min_speed_bottom = sqrt(2*min_E/(constants.m));

s0 = min_speed_bottom;
% the energy is then
E = 0.5*constants.m*s0^2;

z_raw = constants.radius*(1-cos(linspace(0,pi,pi/d_theta)));
speed_raw = sqrt(2*(E-constants.m*constants.g*z_raw)/constants.m);
omega_raw = speed_raw/constants.radius;

theta_raw = acos((constants.radius-z_raw)/constants.radius);
t_raw = [0,cumsum(diff(theta_raw)./omega_raw(1:end-1))];

t = linspace(0,t_raw(end), constants.simTime/constants.dt);
omega = interp1(t_raw,omega_raw,t);
speed = interp1(t_raw,speed_raw,t);

t = [t,t(2:end)+t(end)];
omega = [omega,flip(omega(1:end-1))];
speed = [speed,flip(speed(1:end-1))];
theta = cumtrapz(t,omega);
xr = constants.radius*sin(theta);
zr = constants.radius*(1-cos(theta));
f_centp = constants.m * constants.radius * omega.^2;

thrust_world = [-f_centp.*sin(theta); f_centp.*cos(theta)];

%constants.m*constants.g+

acc_world = thrust_world / constants.m;
vel_world = [s0, 0] + cumtrapz(t,acc_world');

pitch = unwrap(atan2(-thrust_world(1,:),thrust_world(2,:)));
pitch_rate = diff(pitch)./diff(t);
alpha = diff(pitch_rate)./diff(t(1:end-1));

%Start pitch forward
%pitch(1) = 0.1;
pitch_rate = [pitch_rate(1), pitch_rate];
trajectory = [xr' zr' -pitch' (speed.*cos(theta))' (speed.*sin(theta))' -pitch_rate'];

% Transpose nominal thrust to match trajectory shape
u_f_matrix = thrust_world';

% % Min speed to get around loop
% v_start = 7*sqrt(constants.radius);
% 
% % Starting energy
% E_start = 0.5*constants.m*v_start^2;
% 
% % discretize theta
% theta = -linspace(0, 2*pi, 2*pi/d_theta);
% 
% % positions
% x_vec = -constants.radius*sin(theta);
% z_vec = constants.radius - constants.radius*cos(theta);
% % Tangential velocity
% velocity_tan = sqrt(2/constants.m * (E_start + constants.m*constants.g*z_vec));
% % Velocity components
% x_dot_vec = velocity_tan.*cos(theta);
% z_dot_vec = -velocity_tan.*sin(theta);
% d_theta = -velocity_tan/(constants.radius);
% 
% 
% % Populate positions in trajectory
% trajectory = [x_vec', z_vec', theta', x_dot_vec', z_dot_vec', d_theta'];

% @TODO: Populate u_f vectors into matrix.
%u_f_matrix = ones(length(theta), 2) .* [-constants.m*constants.g/2, -constants.m*constants.g/2];

% Do some analysis
% g_max = (v_nominal^2/r)>>>>>>> 772f27931bd4519ee3bb63b737d111b8071e807e/9.81 + 1;
% 
% disp(["Trajectory takes ", t, " seconds. Max acceleration ", g_max, "g"]);

% DEBUG
%figure(2);
%Visualize(trajectory,2);

end

