function [K, u_f] = getNearestKMatrix(x, x_f_matrix, K_matrix, u_f_matrix)
%GETNEARESTKMATRIX Given a depth-stacked K matrix that is size (len(u), len(x), num_fixed_points)
% and a stack of corresponding x_f vectors ( num_fixed_points,len(x))
% and u_f vectors (num_fixed_points, len(u)).

% Calculate x_bar
x_bar_matrix = x - x_f_matrix;

% calculate norms
x_bar_norms = vecnorm(x_bar_matrix'); % size (1,num_fixed,points)

% Find argmin
[y,i] = min(x_bar_norms);

K = K_matrix(:,:,i);
u_f = u_f_matrix(i,:);

end

