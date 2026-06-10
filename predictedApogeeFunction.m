function apogee = predictedApogeeFunction( ...
    altitude,...
    velocity,...
    t_start,...
    burnTime,...
    motorMass,...
    rocketDryMass,...
    cdArea,...
    rho,...
    g,...
    thrustFunc) % <--- Pass the function handle here

dt = 0.02;
h = altitude;
v = velocity;
predT = t_start;

while v > 0 || predT <= burnTime
    if predT <= burnTime
        currentMotorMass = motorMass * (1 - predT/burnTime);
        % Call the actual thrust function using the look-ahead time!
        thrust = thrustFunc(predT); 
    else
        currentMotorMass = 0;
        thrust = 0;
    end
    m = rocketDryMass + currentMotorMass;

    drag = 0.5 * rho * v^2 * cdArea;
    a = (thrust - drag - m*g) / m;

    v = v + a*dt;
    h = h + v*dt;
    predT = predT + dt;
end
apogee = h;
end