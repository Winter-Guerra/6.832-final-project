function [x] = quadrotorDynamics2d (x, u, constants)

% State is defined as [x z theta vx vz w]
% Input is defined as [F_1 F_2], where F_i is the thrust from a specific
% motor. F_i >= 0 should be ensured in this function.
% 
% Define the width of the quadrotor (distance between motors)
quad_baseline = 0.1;

% Make sure that thrust and torque are dynamically feasible.
u_clipped = max(u, 0);

% calculate body torque
tau_body = (u_clipped(1) - u_clipped(2))*quad_baseline/2;

% Calculate total thrust
thrust = sum(u_clipped);

%Update the state
w_dot = inv(constants.J) * tau_body;
x(6) = x(6) + w_dot * constants.dt;

x(3) = x(3) + x(6) * constants.dt;

vz_dot = constants.g + thrust * sin(x(3));
vx_dot = thrust * cos(x(3));

x(4) = x(4) + vx_dot * constants.dt;
x(5) = x(5) + vz_dot * constants.dt;

x(1) = x(1) + x(4) * constants.dt;
x(2) = x(2) + x(5) * constants.dt;

end