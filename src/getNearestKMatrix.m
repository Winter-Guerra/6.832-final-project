function [K, u_f, x_f, k_idx] = getNearestKMatrix(x, x_f_matrix, K_matrix, u_f_matrix, last_k_idx)
%GETNEARESTKMATRIX Given a depth-stacked K matrix that is size (len(u), len(x), num_fixed_points)
% and a stack of corresponding x_f vectors ( num_fixed_points,len(x))
% and u_f vectors (num_fixed_points, len(u)).

% Calculate x_bar
x_bar_matrix = x - x_f_matrix;

% Only compare q (no qd).
%x_bar_matrix = x_bar_matrix(:, 1:2);

x_bar_matrix = x_bar_matrix(last_k_idx:end, 1:2);

% calculate norms
x_bar_norms = vecnorm(x_bar_matrix'); % size (1,num_fixed,points)

% Find argmin
[y,i] = min(x_bar_norms);

% If the jump is extremely large, this is probably a bad fit.
if (i > 100)
i=2;
end

k_idx = i + last_k_idx - 1;
%k_idx = i ;


K = K_matrix(:,:,k_idx);
u_f = u_f_matrix(k_idx,:);
x_f = x_f_matrix(k_idx,:);

end

