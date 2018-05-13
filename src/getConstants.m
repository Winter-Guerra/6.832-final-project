function [constants] = getConstants()
% constants.dt = sim_dt;
constants.m  = 1;
constants.g  = -9.8;
constants.J  = 5e-2;
constants.tmax = 3*constants.g;
constants.dt = 0.01;
constants.radius = 1;
% Define the width of the quadrotor (distance between motors)
constants.baseline = 0.1;
constants.radius = 1;

% Publish jacobians for system
[constants.df_dx, constants.df_du, constants.x_sym, constants.u_sym] = getSystemJacobians(constants);
end

