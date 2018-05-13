function [A,B] = getLinearizedDynamics( x_f, u_f, constants)
%LINEARIZESYSTEMAROUNDPOINT 
% Takes in fixed point x_f, u_f and outputs linearized system A,B such that
% dot(x_bar) = A*x_bar + B*u_bar
% Where x_bar = x-x_f; u_bar = u-u_f

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

% Evaluate jacobians at x_f and u_f

% For df_dx_at_x_f, set u=0 since the system is control affine.
df_dx_at_x_f = subs(df_dx, ...
    {x_sym(1),x_sym(2),x_sym(3),x_sym(4),x_sym(5),x_sym(6),u_sym(1),u_sym(2)}, ...
    {x_f(1)  ,x_f(2)  ,x_f(3)  ,x_f(4)  ,x_f(5)  ,x_f(6)  ,u_f(1)  ,u_f(2)});


%disp(df_dx)

df_du_at_x_f = subs(df_du, ...
    {x_sym(1),x_sym(2),x_sym(3),x_sym(4),x_sym(5),x_sym(6),u_sym(1),u_sym(2)}, ...
    {x_f(1)  ,x_f(2)  ,x_f(3)  ,x_f(4)  ,x_f(5)  ,x_f(6)  ,u_f(1)  ,u_f(2)});

%disp(df_du)


A = double(df_dx_at_x_f);
B = double(df_du_at_x_f);

end

