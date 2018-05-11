function [ ExpX ] = Exp( x )
%EXP returns the exponential map of x
%   x is a vector of three elements

% ExpX = expm(skew(x));
if all(x == 0)
    ExpX = eye(3);
    return
end
normx = norm(x);
ExpX = eye(3) + sin(normx)/normx*skew(x) ...
    + (1-cos(normx))/normx^2*skew(x)^2;

end

