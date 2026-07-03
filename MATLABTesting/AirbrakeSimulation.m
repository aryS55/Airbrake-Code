clear;
clc;

%% PARAMETERS
targetApogee = 228.6;     % m (750 ft Goal)
rocketCd = 0.3099823057;
rocketArea = 0.00246176;
rocketDryMass = 0.420;

% Updated to AeroTech E24 Specs
motorMass = 0.071;        % 71 grams
burnTime = 3.5;           % 2 seconds
totalImpulse = 49.6;      % Newton-seconds
rho = 1.225;
g = 9.81;
dt = 0.01;
maxTime = 60;

%% MG90S SERVO SPECIFICATIONS (Under Load Model)
% Standard MG90S at 4.8V-6V: No-load speed approx 0.1 s/60 deg -> ~10.47 rad/s (600 deg/s)
% Stall Torque: ~1.8 kg-cm = 0.176 N-m
servoMaxSpeed_noload = 600;       % degrees / second
servoStallTorque = 0.176;         % N-m
finChord = 0.015;                 % meters (Assumed average distance from hinge to center of pressure)

%% FIN DATA (Scaled up slightly so they are powerful enough to modulate!)
angles = [0 5 10 15 20 25 30 35 38];
finCd = [0 .123 .249 .333 .393 .443 .492 .554 .560];
finArea = [0 .0000645 .000129 .000193548 .000251612 .000309677 .000412902 .000412902 .000438709] * 2.5; 

%% STATE VARIABLES
t = 0;
velocity = 0;
altitude = 0;
currentFinAngle = 0; % Track actual physical position of airbrakes

maxSteps = ceil(maxTime/dt);
timeHist = zeros(maxSteps,1);
altHist = zeros(maxSteps,1);
velHist = zeros(maxSteps,1);
predHist = zeros(maxSteps,1);
angleHist = zeros(maxSteps,1); 
targetAngleHist = zeros(maxSteps,1); % Track what the controller *wanted*

% New History Tracking Arrays
totalDragHist = zeros(maxSteps,1);
airbrakeDragHist = zeros(maxSteps,1);
accelHist = zeros(maxSteps,1);
step = 1;

deployTimes = [];
deployAngles = [];
previousFinAngle = 0;

%% MAIN LOOP
while altitude >= 0 && t < maxTime
    %% MASS
    if t <= burnTime
        currentMotorMass = motorMass*(1 - t/burnTime);
    else
        currentMotorMass = 0;
    end
    mass = rocketDryMass + currentMotorMass;
    
    %% THRUST
    thrust = ThrustFunction(t, burnTime, totalImpulse);
    
    %% BASE DRAG
    rocketDrag = DragFunction(rocketCd, velocity, rocketArea, rho);
    
    %% DYNAMIC CONTROLLER (Determines Target Angle)
    targetFinAngle = 0; 
    predictedApogee = altitude;
    
    for k = 1:length(angles)
        angle = angles(k);
        idx = find(angles == angle);
        totalCdArea = rocketCd*rocketArea + 3*finCd(idx)*finArea(idx);
        
        % Predict apogee passing current motor values and function handle
        predApogee = predictedApogeeFunction(...
            altitude, velocity, t, burnTime, motorMass, rocketDryMass, totalCdArea, rho, g, totalImpulse);
        
        if predApogee <= targetApogee
            targetFinAngle = angle;
            predictedApogee = predApogee;
            break;
        end
        
        if angle == 38
            targetFinAngle = 38;
            predictedApogee = predApogee;
        end
    end
    
    %% SERVO ACTUATION UNDER LOAD & FLAP TRANSITION TIME
    % 1. Compute current drag on 1 flap to estimate hinge moment
    [c_d_current, area_current] = InterpolateFinProps(currentFinAngle, angles, finCd, finArea);
    singleFlapDrag = DragFunction(c_d_current, velocity, area_current, rho); 
    
    % Hinge Moment (Torque = Force * distance to aerodynamic center)
    hingeTorque = singleFlapDrag * finChord; 
    
    % 2. Linear Servo Speed Derating Model: Speed = MaxSpeed * (1 - Torque/StallTorque)
    if hingeTorque < servoStallTorque
        actualServoSpeed = servoMaxSpeed_noload * (1 - (hingeTorque / servoStallTorque));
    else
        actualServoSpeed = 0; % Aero loads stalled the servo completely
    end
    
    % 3. Actuate the fin toward the target angle over timestep dt
    angleError = targetFinAngle - currentFinAngle;
    maxDegreesThisStep = actualServoSpeed * dt;
    
    if abs(angleError) <= maxDegreesThisStep
        currentFinAngle = targetFinAngle; % Arrived at target
    else
        currentFinAngle = currentFinAngle + sign(angleError) * maxDegreesThisStep; % Transitioning...
    end
    
    %% AIRBRAKE DRAG AT TRANSIENT ANGLE
    % Compute drag based on the physical intermediate flap position
    [c_d_transient, area_transient] = InterpolateFinProps(currentFinAngle, angles, finCd, finArea);
    airbrakeDrag = 3 * DragFunction(c_d_transient, velocity, area_transient, rho);
    
    %% LOG DEPLOYMENT CHANGES (Based on target controller output)
    if targetFinAngle ~= previousFinAngle
        deployTimes(end+1) = t;
        deployAngles(end+1) = targetFinAngle;
        previousFinAngle = targetFinAngle;
    end
    
    %% DYNAMICS
    netForce = thrust - rocketDrag - airbrakeDrag - mass*g;
    acceleration = netForce/mass;
    velocity = velocity + acceleration*dt;
    altitude = altitude + velocity*dt;
    
    %% STORE DATA
    timeHist(step) = t;
    altHist(step) = altitude;
    velHist(step) = velocity;
    predHist(step) = predictedApogee;
    angleHist(step) = currentFinAngle;     % Track Actual position
    targetAngleHist(step) = targetFinAngle; % Track Target setting
    
    % Store new parameters
    totalDragHist(step) = rocketDrag + airbrakeDrag;
    airbrakeDragHist(step) = airbrakeDrag;
    accelHist(step) = acceleration;
    
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
velHist = velHist(1:step-1);
predHist = predHist(1:step-1);
angleHist = angleHist(1:step-1);
targetAngleHist = targetAngleHist(1:step-1);
totalDragHist = totalDragHist(1:step-1);
airbrakeDragHist = airbrakeDragHist(1:step-1);
accelHist = accelHist(1:step-1);

%% PLOTS (2x2 Layout)
figure('Position', [50, 50, 1200, 800])

% Graph 1: Trajectory Profile
subplot(2,2,1)
plot(timeHist,altHist,'LineWidth',2)
hold on
plot(timeHist,predHist,'--','LineWidth',2)
yline(targetApogee,'k:','750 ft Goal')
xlabel('Time (s)')
ylabel('Altitude (m)')
title('Flight Profile')
legend('Actual Altitude', 'Predicted Apogee', 'Target')
grid on

% Graph 2: Actuator Tracking (Target vs Actual Lag)
subplot(2,2,2)
plot(timeHist, targetAngleHist, 'b--', 'LineWidth', 1.5)
hold on
plot(timeHist, angleHist, 'r-', 'LineWidth', 2)
xlabel('Time (s)')
ylabel('Fin Angle (deg)')
title('MG90S Servo Response vs. Controller Target')
legend('Target Command', 'Actual Fin Position')
grid on

% Graph 3: Drag Forces vs Time
subplot(2,2,3)
plot(timeHist, totalDragHist, 'b-', 'LineWidth', 2)
hold on
plot(timeHist, airbrakeDragHist, 'm--', 'LineWidth', 1.5)
xlabel('Time (s)')
ylabel('Force (N)')
title('Aerodynamic Drag Profiles (Inc. Flap Transitions)')
legend('Total Drag Force', 'Airbrake Contribution Only')
grid on

% Graph 4: Acceleration vs Time
subplot(2,2,4)
plot(timeHist, accelHist, 'g-', 'LineWidth', 2)
hold on
yline(0, 'k--', 'Zero Accel Limit')
xlabel('Time (s)')
ylabel('Acceleration (m/s²)')
title('Rocket Acceleration Profiles')
grid on

%% RESULTS
fprintf('\n');
fprintf('Final Apogee = %.2f m (%.2f ft)\n', max(altHist), max(altHist)*3.28084);

%% HELPER FUNCTIONS
function thrust = ThrustFunction(t, burnTime, totalImpulse)
    if t >= 0 && t <= burnTime
        thrust = totalImpulse / burnTime; 
    else
        thrust = 0;
    end
end

function drag = DragFunction(Cd, velocity, area, rho)
    drag = 0.5 * rho * velocity^2 * Cd * area;
end

% Linearly interpolates Cd and Area values for non-discrete, transitioning fin angles
function [Cd_interp, Area_interp] = InterpolateFinProps(currentAngle, angles, finCd, finArea)
    Cd_interp = interp1(angles, finCd, currentAngle, 'linear', 'extrap');
    Area_interp = interp1(angles, finArea, currentAngle, 'linear', 'extrap');
    
    % Enforce boundary safety limits
    Cd_interp = max(0, Cd_interp);
    Area_interp = max(0, Area_interp);
end

function apogee = predictedApogeeFunction(...
    altitude, velocity, t_start, burnTime, motorMass, rocketDryMass, cdArea, rho, g, totalImpulse)
    
    dt_pred = 0.02;
    h = altitude;
    v = velocity;
    predT = t_start;
    
    while v > 0 || predT <= burnTime
        if predT <= burnTime
            m_motor = motorMass * (1 - predT/burnTime);
            thrust = totalImpulse / burnTime; 
        else
            m_motor = 0;
            thrust = 0;
        end
        m = rocketDryMass + m_motor;
        
        drag = 0.5 * rho * v^2 * cdArea;
        a = (thrust - drag - m*g) / m;
        
        v = v + a*dt_pred;
        h = h + v*dt_pred;
        predT = predT + dt_pred;
    end
    apogee = h;
end