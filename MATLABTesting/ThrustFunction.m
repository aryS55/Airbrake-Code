function [thrust] = ThrustFunction(t,burntime,totalImpulse)
%THRUSTFUNCTION Simulates the thrust profile of the rocket motor
%   Calculates the active thrust in Newtons at a specific timestamp.
arguments (Input)
    t,burntime,totalImpulse
end
arguments (Output)
    thrust
end


% Determine thrust based on time step
if t >= 0 && t <= burnTime
    thrust = totalImpulse / burnTime; % Average thrust 
else
    thrust = 0;                       % Motor burnout
end

end