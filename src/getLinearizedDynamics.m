function [A,B] = getLinearizedDynamics( x_f, u_f, c)
%LINEARIZESYSTEMAROUNDPOINT 
% Takes in fixed point x_f, u_f and outputs linearized system A,B such that
% dot(x_bar) = A*x_bar + B*u_bar
% Where x_bar = x-x_f; u_bar = u-u_f

% Check getSystemJacobians.m file for the derivation of the jacobians
% c.df_dx, c.df_du.

% Evaluate jacobians at x_f and u_f

% For df_dx_at_x_f, set u=0 since the system is control affine.
df_dx_at_x_f = subs(c.df_dx, ...
    {c.x_sym(1),c.x_sym(2),c.x_sym(3),c.x_sym(4),c.x_sym(5),c.x_sym(6),c.u_sym(1),c.u_sym(2)}, ...
    {x_f(1)  ,x_f(2)  ,x_f(3)  ,x_f(4)  ,x_f(5)  ,x_f(6)  u_f(1)  ,u_f(2)});


%disp(df_dx)

df_du_at_x_f = subs(c.df_du, ...
    {c.x_sym(1),c.x_sym(2),c.x_sym(3),c.x_sym(4),c.x_sym(5),c.x_sym(6),c.u_sym(1),c.u_sym(2)}, ...
    {x_f(1)  ,x_f(2)  ,x_f(3)  ,x_f(4)  ,x_f(5)  ,x_f(6)  ,u_f(1)  ,u_f(2)});

%disp(df_du)


A = double(df_dx_at_x_f);
B = double(df_du_at_x_f);

end

