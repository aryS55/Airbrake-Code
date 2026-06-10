clear;
clc;

%% PARAMETERS
rocketCd = 0.3099823057;
rocketArea = 0.00246176;
rocketDryMass = 0.420;

% Motor Specs (AeroTech E24)
motorMass = 0.071;        
burnTime = 3.5;           
totalImpulse = 49.6;      

rho = 1.225;
g = 9.81;
dt = 0.01;
maxTime = 60;

%% STATE VARIABLES
t = 0;
velocity = 0;
altitude = 0;
maxSteps = ceil(maxTime/dt);

timeHist = zeros(maxSteps,1);
altHist = zeros(maxSteps,1);
step = 1;

%% MAIN SIMULATION LOOP
while altitude >= 0 && t < maxTime
    %% DYNAMIC MASS
    if t <= burnTime
        currentMotorMass = motorMass * (1 - t/burnTime);
    else
        currentMotorMass = 0;
    end
    mass = rocketDryMass + currentMotorMass;

    %% THRUST
    if t >= 0 && t <= burnTime
        thrust = totalImpulse / burnTime; 
    else
        thrust = 0;
    end

    %% AERODYNAMIC DRAG (Base Rocket Only)
    rocketDrag = 0.5 * rho * velocity^2 * rocketCd * rocketArea;

    %% DYNAMICS
    netForce = thrust - rocketDrag - mass*g;
    acceleration = netForce / mass;

    velocity = velocity + acceleration * dt;
    altitude = altitude + velocity * dt;

    %% STORE DATA
    timeHist(step) = t;
    altHist(step) = altitude;
    step = step + 1;

    %% STOP AT APOGEE
    if velocity <= 0 && t > burnTime
        break;
    end
    t = t + dt;
end

%% TRIM ARRAYS
timeHist = timeHist(1:step-1);
altHist = altHist(1:step-1);

%% PLOT (Altitude vs Time Only)
figure
plot(timeHist, altHist, 'b-', 'LineWidth', 2)
xlabel('Time (s)')
ylabel('Altitude (m)')
title('Rocket Altitude vs Time (Pure Ballistic Flight)')
grid on

%% RESULTS
fprintf('\nPure Ballistic Apogee: %.2f m (%.2f ft)\n', max(altHist), max(altHist)*3.28084);