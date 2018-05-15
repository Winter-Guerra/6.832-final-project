function [RMSE, finalError, positionErrorTimeline] = analyzeTrajectory(simulatedTrajectory, nominalTrajectory, constants)
%analyzeTrajectory 
% Takes in a simulated __loop__ (n, len(x)) and
% Returns RMSE (scalar) and positionErrorTimeline (1,t)

% Figure out which points on simulated trajectory correspond to the loop.
% Defined as the set of points inside of the 2 X=0 crossings.

leftBoundIDX = find(simulatedTrajectory(:,1) > 0, 1, 'first');
rightBoundIDX = find(simulatedTrajectory(:,1) < 0, 1, 'last');


% Get positions
positions = simulatedTrajectory(leftBoundIDX:rightBoundIDX,1:2);
% Center of circle
C = [0 constants.radius];

% Positions as vectors from center of circle
vectors = (positions - C);

% Project positions onto circle
positionsOnCircle = C + vectors / vecnorm(vectors) * constants.radius;

% Find error between positions and closest point on circle.
positionalError = positions - positionsOnCircle;


% positional error
%trajectoryError = trajectory_nominal - simulatedTrajectory;
% Make sure that positional error only contains positional error
%positionalError = trajectoryError(:,1:2);
positionErrorTimeline = vecnorm(positionalError');

RMSE = sqrt( sum(positionErrorTimeline.^2) / numel(positionErrorTimeline) );

finalError = vecnorm(simulatedTrajectory(end:end,1:2) - nominalTrajectory(end:end,1:2));


end

