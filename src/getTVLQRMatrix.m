function [K_matrix] = getTVLQRMatrix(trajectory, u_f_matrix, constants)
%GETTVLQRMATRIX Given a trajectory of size (len(x), n), will output
% K_matrix (len(u), len(x), num_fixed_points)
% u_f_matrix (num_fixed_points, len(u))

% Sizes
[n , len_x] = size(trajectory);
len_u = 2;

K_matrix = zeros(len_u, len_x, n);
u_f_matrix = zeros(n, len_u);

% Iterate through each fixed point and run LQR.
for i = 1:n
    % Get u_f and x_f from inputs.
    x_f = trajectory(i,:);
    u_f = u_f_matrix(i,:);
    
    % Compute and save K.
    K = lqrPositionController(x_f, u_f, constants);
    K(:,:,i) = K;
end


end

