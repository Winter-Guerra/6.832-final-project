function [ x_hat ] = skew( x )
%SSM returns the skew symetric matrix constructed from x
%   x is a one dimensional 3 element vector

x_hat = [0 -x(3) x(2) ; x(3) 0 -x(1) ; -x(2) x(1) 0 ];

end

