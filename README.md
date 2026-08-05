# Model Rocket Active Airbrake System

An active drag-brake control system for a model rocket, designed to precisely hit a target apogee by dynamically deploying airbrake fins during coast. The repository contains both the MATLAB flight-simulation/design tools used to size and tune the system, and the Arduino firmware that flies it.

## Overview

Model rocket motors have manufacturing variance, and factors like launch-day air density and rail angle mean a rocket's actual apogee will always deviate somewhat from a purely pre-calculated estimate. This project counteracts that by using three deployable airbrake fins, driven by a micro servo, that increase drag during coast flight to trim the rocket down to a **750 ft (228.6 m) target apogee**.

The system has two halves:

- **MATLAB simulation suite** — models rocket flight with and without airbrakes, characterizes fin drag at different deployment angles, includes a realistic servo actuation/derating model, and predicts apogee in real time using a forward-integration look-ahead algorithm.
- **Arduino flight controller** (`Airbrake_code_final1.ino`) — runs on the rocket itself, reading altitude from a BMP280 barometer, estimating velocity, detecting launch and apogee, and commanding the airbrake servo during coast.

## Repository Contents

| File | Description |
|---|---|
| `AirbrakeSimulation.m` | Main MATLAB simulation. Flies the rocket with a closed-loop airbrake controller that selects a fin angle every timestep based on a predicted-apogee look-ahead, models servo speed under aerodynamic hinge-moment loading, and plots the full flight profile. |
| `NoAirbrakeTest.m` | Baseline "no airbrake" ballistic flight simulation, used to sanity-check motor/mass parameters and establish the uncontrolled apogee. |
| `predictedApogeeFunction.m` | Standalone function that forward-integrates the equations of motion from the current state to predict apogee, given a thrust function handle. Used for validating the controller logic outside the main sim. |
| `ThrustFunction.m` | Simple average-thrust motor model (total impulse / burn time) used by the simulations. |
| `DragFunction.m` | Basic drag force calculation, `F_d = 0.5 * ρ * Cd * A * v²`. |
| `Airbrake_Cd_A__Sheet1.csv` | Wind-tunnel/empirical data of drag coefficient (Cd), frontal area, and drag force for the airbrake fins and rocket body at deployment angles from 0°–40°. Used to build the Cd/area interpolation tables in the simulation. |
| `Airbrake_code_final1.ino` | Arduino firmware for the onboard flight computer: reads a BMP280 barometer, detects launch and apogee, predicts apogee during coast, and drives the airbrake servo. Logs final apogee and deployment status to EEPROM. |

## How the Simulation Works

1. **Baseline flight** (`NoAirbrakeTest.m`) integrates the rocket's equations of motion (thrust, drag, weight) forward in time with no airbrakes, to find the natural apogee for the given motor and rocket mass.
2. **Controlled flight** (`AirbrakeSimulation.m`) adds a closed-loop controller that, at every timestep during coast:
   - Sweeps candidate fin angles (0°–38°) and, for each, uses `predictedApogeeFunction` to forward-simulate the rest of the flight assuming that drag configuration.
   - Selects the smallest fin angle that is predicted to bring the rocket to at or below the target apogee.
   - Converts that target angle into a physical fin position, accounting for **MG90S servo dynamics**: no-load slew speed, stall torque, and a hinge-moment torque calculated from real-time aerodynamic loading on the fins (so the servo realistically slows down or stalls under high drag loads).
   - Applies the resulting airbrake drag to the equations of motion for that timestep.
3. Results are plotted: altitude vs. time (actual vs. predicted apogee), commanded vs. actual fin angle (showing servo lag), drag force breakdown, and acceleration profile.

## How the Firmware Works

The Arduino sketch (`Airbrake_code_final1.ino`) mirrors the simulation's control philosophy in a lightweight, real-time-safe form suitable for a microcontroller:

1. **Sensing** — Reads altitude from a BMP280 barometric sensor (~20 Hz loop) and derives a smoothed velocity estimate via a simple exponential filter.
2. **Launch detection** — Watches for a sustained upward velocity threshold to confirm liftoff (debounced over several samples to reject pad vibration).
3. **Coast-phase apogee prediction** — Once velocity is positive and acceleration has gone negative (motor burnout), predicts apogee ballistically from current velocity and altitude.
4. **Airbrake control** — Maps the predicted-apogee error against the target apogee to a servo angle (0°–110°) with a small deadband, deploying the fins proportionally to trim excess altitude.
5. **Apogee detection & logging** — Detects apogee via sustained negative velocity, retracts the airbrakes, and logs the max altitude and deployment flag to EEPROM for post-flight review.

## Getting Started

### MATLAB Simulations
1. Open MATLAB and add this repository to your path.
2. Run `NoAirbrakeTest.m` to see the baseline (uncontrolled) apogee for your motor/rocket parameters.
3. Run `AirbrakeSimulation.m` to see the airbrake-controlled flight, including servo response and drag plots.
4. Adjust rocket parameters (`rocketCd`, `rocketArea`, `rocketDryMass`), motor parameters (`motorMass`, `burnTime`, `totalImpulse`), or `targetApogee` at the top of each script to model your own rocket.

### Arduino Firmware
1. Install the required libraries: `Adafruit_BMP280`, `Servo`, `EEPROM` (built-in), `Wire` (built-in).
2. Wire the BMP280 (I2C, address `0x76`) and a servo signal wire to pin 9.
3. Flash `Airbrake_code_final1.ino` to your flight computer.
4. Update `targetApogee` and the `mapApogeeToServo` mapping to match your rocket's characterized performance from the MATLAB simulation and Cd/area data.

## Hardware

- **Barometer:** Adafruit BMP280
- **Actuator:** MG90S micro servo
- **Airbrake fins:** 3x deployable fins, 0°–38° range, characterized in `Airbrake_Cd_A__Sheet1.csv`
- **Motor (test config):** AeroTech E-class motor (parameters configurable in the MATLAB scripts)

## Notes / Future Work

- The onboard firmware's apogee prediction (`PredictAlt`) uses a simplified ballistic estimate (`v²/19.6`), whereas the MATLAB simulation uses a full forward-integration model — these could be unified for closer sim-to-flight correlation.
- Cd/area interpolation for intermediate fin angles is currently linear (`interp1`); a higher-fidelity aerodynamic model could improve prediction accuracy.
- No apogee-detection redundancy (e.g., accelerometer fusion) is currently implemented in firmware; barometric-only sensing is used.

## Disclaimer

This is an experimental hobby rocketry project. Always follow your national rocketry association's safety code, use certified motors within your certification level, and never fly an active airbrake system without extensive ground testing and simulation validation.
