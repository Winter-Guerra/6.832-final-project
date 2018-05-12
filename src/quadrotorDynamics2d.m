function [state] = quadrotorDynamics2d (state, inputs, constants)

% State is defined as [x z theta vx vz w]

%Update the state
w_dot = inv(constants.J) * inputs.torque;
state(6) = state(6) + w_dot * constants.dt;

state(3) = state(3) + state(6) * constants.dt;

vz_dot = constants.g + inputs.thrust * sin(state(3));
vx_dot = inputs.thrust * cos(state(3));

state(4) = state(4) + vx_dot * constants.dt;
state(5) = state(5) + vz_dot * constants.dt;

state(1) = state(1) + state(4) * constants.dt;
state(2) = state(2) + state(5) * constants.dt;

end