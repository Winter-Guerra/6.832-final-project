% Quadrotor Dynamics
function [state] = quadrotorDynamics (state, inputs, constants)

% Update the state
w_dot = inv(constants.J) * (inputs.torque - cross(state.angularVelocity, constants.J * state.angularVelocity));
state.angularVelocity = state.angularVelocity + w_dot * constants.dt;

R_dot = Exp(state.angularVelocity * constants.dt);
state.rotation = state.rotation * R_dot;

v_dot = constants.g + state.rotation * inputs.thrust/constants.m;
%v_dot = state.rotation * inputs.thrust/constants.m;
state.velocity = state.velocity + v_dot * constants.dt;

p_dot = state.velocity;
state.position = state.position + p_dot * constants.dt;

end