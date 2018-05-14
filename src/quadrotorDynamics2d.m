function [x, x_dot] = quadrotorDynamics2d (x, u, constants)

% State is defined as [x z theta vx vz w]
% Input is defined as [F_1 F_2], where F_i is the thrust from a specific
% motor. F_i >= 0 should be ensured OUTSIDE of this function. 


% calculate body torque
tau_body = (u(1) - u(2))*constants.baseline/2;

% Calculate total thrust
acceleration = sum(u) / constants.m;

% Calculate x_dot
w_dot = tau_body / constants.J;
vz_dot = constants.g + acceleration * cos(x(3))
vx_dot = acceleration * sin(x(3))


x_dot = [x(4), x(5), x(6), vx_dot, vz_dot, w_dot];

% Propagate state using dt*x_dot
%x = x + x_dot*constants.dt;

x(6) = x(6) + w_dot * constants.dt;
x(3) = x(3) + x(6) * constants.dt;
x(4) = x(4) + vx_dot * constants.dt;
x(5) = x(5) + vz_dot * constants.dt; 
x(1) = x(1) + x(4) * constants.dt;
x(2) = x(2) + x(5) * constants.dt;


end