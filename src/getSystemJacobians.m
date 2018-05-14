function [df_dx, df_du, x_sym, u_sym] = getSystemJacobians(constants)
%GETSYSTEMJACOBIANS Summary of this function goes here
%   Detailed explanation goes here

% A is shape (len(x), len(x))
% B is shape (len(x), len(x))

len_x = 6;
len_u = 2;

% Get a symbolic representation of the system inputs.
x_sym = sym('x', [len_x,1]);
u_sym = sym('u', [len_u,1]);

% Get system equations for state.
[x, x_dot] = quadrotorDynamics2d (x_sym, u_sym, constants);
f_system = x_dot; % f = x_dot

% x_dot = f(x,u)
% x_dot ~= f(x_f, u_f) + (df/dx|x=x_f, u=u_f)*x_bar + (df/du|x=x_f, u=u_f)*u_bar

% Get jacobians of system
df_dx = jacobian(f_system, x_sym);
df_du = jacobian(f_system, u_sym);
end

