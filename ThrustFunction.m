function [thrust] = ThrustFunction(t)
%THRUSTFUNCTION Simulates the thrust profile of the rocket motor
%   Calculates the active thrust in Newtons at a specific timestamp.
arguments (Input)
    t
end
arguments (Output)
    thrust
end

% Motor specifications from the datasheet
burnTime = 3.5;       % seconds
totalImpulse = 49.6;   % Newton-seconds

% Determine thrust based on time step
if t >= 0 && t <= burnTime
    thrust = totalImpulse / burnTime; % Average thrust 
else
    thrust = 0;                       % Motor burnout
end

end