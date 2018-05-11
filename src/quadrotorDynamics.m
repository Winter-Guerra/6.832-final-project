% Quadrotor Dynamics
function [state] = quadrotorDynamics (state, inputs, constants)

% compute the updates
p_dot = state.velocity;
v_dot = constants.g + state.rotation * inputs.thrust;
R_dot = Exp(state.angularVelocity * constants.dt);
w_dot = inv(constants.J) * (inputs.torque - cross(state.angularVelocity, constants.J * state.angularVelocity));

% Propogate the state
state.position = state.position + p_dot * constants.dt;
state.rotation = state.rotation * R_dot;
state.velocity = state.velocity + v_dot * constants.dt;
state.angularVelocity = state.angularVelocity + w_dot * constants.dt;

%
%w_dot = inv(J) * (eta - w x J w)

end