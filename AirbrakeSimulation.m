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

%% FIN DATA (Scaled up slightly so they are powerful enough to modulate!)
angles = [0 5 10 15 20 25 30 35 38];
finCd = [0 .123 .249 .333 .393 .443 .492 .554 .560];
finArea = [0 .0000645 .000129 .000193548 .000251612 .000309677 .000412902 .000412902 .000438709] * 2.5; 

%% STATE VARIABLES
t = 0;
velocity = 0;
altitude = 0;
maxSteps = ceil(maxTime/dt);

timeHist = zeros(maxSteps,1);
altHist = zeros(maxSteps,1);
velHist = zeros(maxSteps,1);
predHist = zeros(maxSteps,1);
angleHist = zeros(maxSteps,1); 

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
    
    %% DYNAMIC CONTROLLER
    finAngle = 0; 
    predictedApogee = altitude;
    
    for k = 1:length(angles)
        angle = angles(k);
        idx = find(angles == angle);
        totalCdArea = rocketCd*rocketArea + 3*finCd(idx)*finArea(idx);
        
        % Predict apogee passing current motor values and function handle
        predApogee = predictedApogeeFunction(...
            altitude, velocity, t, burnTime, motorMass, rocketDryMass, totalCdArea, rho, g, totalImpulse);
        
        if predApogee <= targetApogee
            finAngle = angle;
            predictedApogee = predApogee;
            break;
        end
        
        if angle == 38
            finAngle = 38;
            predictedApogee = predApogee;
        end
    end
    
    %% AIRBRAKE DRAG
    idx = find(angles == finAngle);
    airbrakeDrag = 3 * DragFunction(finCd(idx), velocity, finArea(idx), rho);
    
    %% LOG DEPLOYMENT CHANGES
    if finAngle ~= previousFinAngle
        deployTimes(end+1) = t;
        deployAngles(end+1) = finAngle;
        previousFinAngle = finAngle;
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
    angleHist(step) = finAngle;
    
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
for i = 1:length(deployTimes)
    xline(deployTimes(i),':r','LineWidth',1.2);
end
xlabel('Time (s)')
ylabel('Altitude (m)')
title('Flight Profile')
legend('Actual Altitude', 'Predicted Apogee', 'Target')
grid on

% Graph 2: Dynamic Airbrake Angle Tracking
subplot(2,2,2)
plot(timeHist, angleHist, 'r-', 'LineWidth', 2)
xlabel('Time (s)')
ylabel('Fin Angle (deg)')
title('Dynamic Airbrake Deflection')
grid on

% Graph 3: Drag Forces vs Time (New)
subplot(2,2,3)
plot(timeHist, totalDragHist, 'b-', 'LineWidth', 2)
hold on
plot(timeHist, airbrakeDragHist, 'm--', 'LineWidth', 1.5)
for i = 1:length(deployTimes)
    xline(deployTimes(i),':r','LineWidth',1.2);
end
xlabel('Time (s)')
ylabel('Force (N)')
title('Aerodynamic Drag Profiles')
legend('Total Drag Force', 'Airbrake Contribution Only')
grid on

% Graph 4: Acceleration vs Time (New)
subplot(2,2,4)
plot(timeHist, accelHist, 'g-', 'LineWidth', 2)
hold on
yline(0, 'k--', 'Zero Accel Limit')
for i = 1:length(deployTimes)
    xline(deployTimes(i),':r','LineWidth',1.2);
end
xlabel('Time (s)')
ylabel('Acceleration (m/s²)')
title('Rocket Acceleration Profiles')
grid on

%% RESULTS
fprintf('\n');
fprintf('Final Apogee = %.2f m (%.2f ft)\n', max(altHist), max(altHist)*3.28084);
fprintf('\nDeployment Events\n');
fprintf('-----------------\n');
for i = 1:length(deployTimes)
    fprintf('t = %.2f s   angle = %d deg\n', deployTimes(i), deployAngles(i));
end

%% LOCAL HELPER FUNCTIONS (Placed at bottom for standalone performance)
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