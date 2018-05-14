function [u] = controlEnergy (x, xd, constants)
   
   Fz = constants.m* (-constants.g + (xd(5) - x(5))/constants.dt)
   Fx = constants.m* (xd(4) - x(4))/constants.dt
   
   thrust = vecnorm([Fx Fz]);
   
   tau = constants.J * (xd(6) - x(6)) / constants.dt;
   
   F1 = 0.5 * (thrust + 2*tau/ constants.baseline);
   F2 = thrust - F1;

   u = [F1, F2];   
end