function [u,t, x0] = controllerEnergy(x,t, constants)

g = -constants.g;
radius = constants.radius;
J = constants.J;

min_speed_top = sqrt(g*radius);
min_omega_top = min_speed_top/radius;
min_KE_l_top = 0.5* constants.m *min_speed_top^2;
PE_top = constants.m*g*(2*radius);
min_E = min_KE_l_top + PE_top;
min_speed_bottom = sqrt(2*min_E/(constants.m));

k = 1;
s0 = k * min_speed_bottom + 2;
% the energy is then
E = 0.5*constants.m*s0^2;

z_raw = radius*(1-cos(linspace(0,pi,500)));
speed_raw = sqrt(2*(E-constants.m*g*z_raw)/constants.m);
omega_raw = speed_raw/radius;

theta_raw = acos((radius-z_raw)/radius);
t_raw = [0,cumsum(diff(theta_raw)./omega_raw(1:end-1))];

t = linspace(0,t_raw(end),1000);
omega = interp1(t_raw,omega_raw,t);

t = [t,t(2:end)+t(end)];
omega = [omega,flip(omega(1:end-1))];
theta = cumtrapz(t,omega);
xr = radius*sin(theta);
zr = radius*(1-cos(theta));
f_centp = constants.m * radius * omega.^2;

thrust_world = [-f_centp.*sin(theta); -g+f_centp.*cos(theta)];

pitch = unwrap(atan2(-thrust_world(1,:),thrust_world(2,:)));
pitch_rate = diff(pitch)./diff(t);
alpha = diff(pitch_rate)./diff(t(1:end-1));
tau = J*alpha;

x0 = [0.0 0.0 0 s0 0.0 -pitch_rate(1)];
tau = [0, 0, -tau];

thrust= vecnorm(thrust_world)';
thrust(2);

F1 = 0.5 * (vecnorm(thrust_world)' + 2*tau'./ constants.baseline);
F2 = vecnorm(thrust_world)' - F1;

F1(1);
F2(2);

u = [F1, F2];

end