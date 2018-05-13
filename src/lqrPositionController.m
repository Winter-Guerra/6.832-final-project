function [u] = lqrPositionController(p_f, x, constants)
%LQRPOSITIONCONTROLLER controller for hovering around a fixed point p_f.
% Assumes level hover.

% Find stable fixed point
x_f = [p_f 0 0 0];
% Nominal thrust should counteract gravity
u_f = [-constants.m*constants.g/2, -constants.m*constants.g/2];

%u_f = [0 0];

% Get linearized dynamics
[A,B] = getLinearizedDynamics(x_f, u_f, constants);

% Pick a reasonable positive semi-definite state cost matrix Q
Q = diag([10,10,1,1,1,1]);

% Pick a reasonable positive semi-definite input cost matrix R
R = eye(2);

% Make N (for off diagonal terms) zero, since this is normally sufficient.
N = zeros(6,2);

% Use LQR to get a control law.
[K,S,e] = lqr(A,B,Q,R,N);


x_bar = x - x_f;

u = -K*x_bar' + u_f';

end

