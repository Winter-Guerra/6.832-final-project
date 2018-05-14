function [K, u_f] = lqrPositionController(x_f, u_f, constants)
%LQRPOSITIONCONTROLLER controller for hovering around a fixed point p_f.
% Assumes level hover.

%u_f = [0 0];

% Get linearized dynamics
[A,B] = getLinearizedDynamics(x_f, u_f, constants);

% Pick a reasonable positive semi-definite state cost matrix Q
Q = diag([5,10,10,15,15,5]);

% Pick a reasonable positive semi-definite input cost matrix R
R = eye(2);

% Make N (for off diagonal terms) zero, since this is normally sufficient.
N = zeros(6,2);

% Use LQR to get a control law.
[K,S,e] = lqr(A,B,Q,R,N);


%x_bar = x - x_f;

%u = -K*x_bar' + u_f';

end

