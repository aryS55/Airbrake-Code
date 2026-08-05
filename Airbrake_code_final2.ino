#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <Servo.h>
#include <EEPROM.h>
#include <SPI.h>
#include <SD.h>

// ============================================================
// HARDWARE OBJECTS
// ============================================================
Servo airbrakeServo;
const int servoPin = 9;
const int chipSelect = 10;   // SD card CS pin -- change to match your wiring

Adafruit_BMP280 bmp;
File logFile;

// ============================================================
// SENSOR / STATE VARIABLES
// ============================================================
float prevAlt;
float prevVel = 0;
float cMaxAlt = 0;
float startingAlt = 0;       // meters, set in setup() after BMP init

// Target apogee -- 750 ft goal, converted to meters AGL to match MATLAB (SI units)
const float targetApogeeAGL_m = 750.0 * 0.3048;   // 228.6 m
float targetApogee_m = 0;                          // absolute (startingAlt + target), set in setup()

// Loop interval in seconds -- matches MATLAB dt used in predictedApogeeFunction stepping cadence
const float dt = 0.05;   // ~20 Hz, as in original code

int negativeVelCount = 0;
bool finsDeployed = false;
bool launched = false;
int launchCounter = 0;

// ============================================================
// ALTITUDE SMOOTHING (BMP280 raw noise is ~1 ft / ~0.3m)
// Simple moving average, filtered BEFORE differentiating for velocity.
// Filtering after differentiating (like the original code did on velocity
// only) doesn't fix noisy position data going INTO the derivative.
// ============================================================
const int ALT_BUFFER_SIZE = 5;   // 5 samples @ 20Hz = 250ms window
float altBuffer[ALT_BUFFER_SIZE];
int altBufIndex = 0;
float altSum = 0;

float smoothAltitude(float rawAlt) {
  altSum -= altBuffer[altBufIndex];
  altBuffer[altBufIndex] = rawAlt;
  altSum += rawAlt;
  altBufIndex = (altBufIndex + 1) % ALT_BUFFER_SIZE;
  return altSum / ALT_BUFFER_SIZE;
}

// ============================================================
// SERVO LAG COMPENSATION
// Instead of simulating the servo ramp INSIDE the apogee predictor
// (expensive nested loop, risky on a Nano), we cheaply dead-reckon
// the rocket's state forward by the servo's typical lag time using
// basic kinematics (one multiply-add, no loop), then predict from
// THAT projected state. This makes the controller command the next
// angle a bit early, roughly compensating for servo travel time.
// Tune SERVO_LAG_S to whatever you measure from the actualAngle_deg
// log column vs targetAngle_deg (time to settle after a command).
// ============================================================
const float SERVO_LAG_S = 0.35;   // seconds, tune from flight/bench data

int lastAngleIndex = 0;   // tracks last commanded angle index for fast local search

const int EEPROM_APOGEE_ADDR = 0;
const int EEPROM_FINS_ADDR = 4;

// ============================================================
// ROCKET / AERO CONSTANTS  (from NoAirbrakeTest.m / AirbrakeSimulation.m)
// ============================================================
const float rocketCd       = 0.3099823057;
const float rocketArea     = 0.00246176;   // m^2
const float rocketDryMass  = 0.420;        // kg (dry mass, coast phase -- no motor mass left)
const float rho            = 1.225;        // kg/m^3
const float g              = 9.81;         // m/s^2

// ============================================================
// FIN LOOKUP TABLE (from AirbrakeSimulation.m, single-fin Cd/Area, *2.5 scale factor baked in)
// Stored in PROGMEM (flash) instead of RAM -- these never change during
// flight, and RAM is the scarce resource on a Nano. Access via
// pgm_read_float(&tableName[i]) instead of tableName[i].
// ============================================================
const int   NUM_ANGLES = 10;
const float finAngles[NUM_ANGLES] PROGMEM = {0, 5, 10, 15, 20, 25, 30, 35, 40, 42.15};
const float finCd[NUM_ANGLES]     PROGMEM = {0, 0.123, 0.249, 0.333, 0.393, 0.443, 0.492, 0.554, 0.625, .655};
const float finArea[NUM_ANGLES]   PROGMEM = {
  0.0,
  0.0000645   * 2.5,
  0.000129    * 2.5,
  0.000193548 * 2.5,
  0.000251612 * 2.5,
  0.000309677 * 2.5,
  0.000412902 * 2.5,
  0.000412902 * 2.5,
  0.000438709 * 2.5,
  0.000438709 * 2.5 // Added padding to match 10 elements
};

// Small helper accessors so the rest of the code doesn't need pgm_read_float() everywhere
inline float getFinAngle(int i) { return pgm_read_float(&finAngles[i]); }
inline float getFinCd(int i)    { return pgm_read_float(&finCd[i]); }
inline float getFinArea(int i)  { return pgm_read_float(&finArea[i]); }

// ============================================================
// MG90S SERVO MODEL (from AirbrakeSimulation.m)
// The servo does NOT snap to the commanded angle -- it takes time,
// and aero load on the fins slows it further. This models that lag
// so currentFinAngle (the REAL physical position) trails targetFinAngle.
// ============================================================
const float servoMaxSpeed_noload = 600.0;   // deg/s, no-load MG90S speed
const float servoStallTorque     = 0.176;   // N-m
const float finChord             = 0.015;   // m, hinge-to-center-of-pressure distance

// ============================================================
// SERVO ANGLE <-> FIN ANGLE MAPPING
// Interpolates the servo angle corresponding to the specific 
// fin angle arrays provided.
// ============================================================
const float servoAngles[NUM_ANGLES] PROGMEM = {0, 9.7, 18.95, 27.98, 37.05, 46.42, 56.51, 68.2, 84.53, 105};

inline float getServoAngle(int i) { return pgm_read_float(&servoAngles[i]); }

inline int finAngleToServo(float finAngle) {
  // Bound checks
  if (finAngle <= getFinAngle(0)) {
    return (int)getServoAngle(0);
  }
  if (finAngle >= getFinAngle(NUM_ANGLES - 1)) {
    return (int)getServoAngle(NUM_ANGLES - 1);
  }
  
  // Linear interpolation between the specific lookup table values
  for (int k = 0; k < NUM_ANGLES - 1; k++) {
    float a0 = getFinAngle(k);
    float a1 = getFinAngle(k + 1);
    if (finAngle >= a0 && finAngle <= a1) {
      float frac = (finAngle - a0) / (a1 - a0);
      float s0 = getServoAngle(k);
      float s1 = getServoAngle(k + 1);
      return (int)(s0 + frac * (s1 - s0));
    }
  }
  return 0;
}

float currentFinAngle = 0;   // actual physical fin position (starts stowed)

// Linear interpolation of Cd/Area for the current (possibly in-between) fin angle
void interpolateFinProps(float angle, float &cdOut, float &areaOut) {
  if (angle <= getFinAngle(0)) {
    cdOut = getFinCd(0); areaOut = getFinArea(0); return;
  }
  if (angle >= getFinAngle(NUM_ANGLES - 1)) {
    cdOut = getFinCd(NUM_ANGLES - 1); areaOut = getFinArea(NUM_ANGLES - 1); return;
  }
  for (int k = 0; k < NUM_ANGLES - 1; k++) {
    if (angle >= getFinAngle(k) && angle <= getFinAngle(k + 1)) {
      float frac = (angle - getFinAngle(k)) / (getFinAngle(k + 1) - getFinAngle(k));
      cdOut   = getFinCd(k)   + frac * (getFinCd(k + 1)   - getFinCd(k));
      areaOut = getFinArea(k) + frac * (getFinArea(k + 1) - getFinArea(k));
      return;
    }
  }
  cdOut = 0; areaOut = 0;
}

// Moves currentFinAngle toward targetFinAngle by one timestep, derating servo
// speed based on the aero hinge torque on a single flap (same model as MATLAB).
void actuateServo(float targetFinAngle, float velocity) {
  float c_d_current, area_current;
  interpolateFinProps(currentFinAngle, c_d_current, area_current);

  float singleFlapDrag = 0.5 * rho * velocity * velocity * c_d_current * area_current;
  float hingeTorque = singleFlapDrag * finChord;

  float actualServoSpeed;
  if (hingeTorque < servoStallTorque) {
    actualServoSpeed = servoMaxSpeed_noload * (1.0 - (hingeTorque / servoStallTorque));
  } else {
    actualServoSpeed = 0;   // aero load has stalled the servo
  }

  float angleError = targetFinAngle - currentFinAngle;
  float maxDegreesThisStep = actualServoSpeed * dt;

  if (abs(angleError) <= maxDegreesThisStep) {
    currentFinAngle = targetFinAngle;
  } else {
    currentFinAngle += (angleError > 0 ? 1 : -1) * maxDegreesThisStep;
  }

  airbrakeServo.write(finAngleToServo(currentFinAngle));
}

// ============================================================
// STARTUP SERVO SWEEP
// Quick self-test: steps the servo through every preset fin angle
// (0 -> 42.15 -> 0), HOLDING each position for holdTime_ms so you can
// actually see/verify each one, rather than blurring through them.
// ============================================================
void startupServoSweep() {
  const int holdTime_ms = 500;   // time to hold at EACH angle

  // Sweep up through every preset angle
  for (int k = 0; k < NUM_ANGLES; k++) {
    airbrakeServo.write(finAngleToServo(getFinAngle(k)));
    delay(holdTime_ms);
  }
  // Sweep back down (skip the top angle, already there)
  for (int k = NUM_ANGLES - 2; k >= 0; k--) {
    airbrakeServo.write(finAngleToServo(getFinAngle(k)));
    delay(holdTime_ms);
  }

  currentFinAngle = 0;
  airbrakeServo.write(0);
}

// ============================================================
// PREDICTED APOGEE SIMULATION
// Mirrors predictedApogeeFunction.m / the inline sim in AirbrakeSimulation.m,
// but simplified for the coast phase (motor already burned out -> thrust = 0,
// mass = rocketDryMass only). This matches the real flight condition, since
// the controller only runs its prediction while vel > 0 && acc < 0 (coasting).
// ============================================================
float predictApogee(float alt, float vel, float cdArea) {
  const float dt_pred = 0.02;   // same look-ahead step as MATLAB
  float h = alt;
  float v = vel;

  while (v > 0) {
    float drag = 0.5 * rho * v * v * cdArea;
    float a = (-drag - rocketDryMass * g) / rocketDryMass;
    v = v + a * dt_pred;
    h = h + v * dt_pred;
  }
  return h;
}

// ============================================================
// CONTROLLER: pick the smallest fin angle that drives predicted
// apogee at or below target. Same logic as the for-loop in
// AirbrakeSimulation.m, but searches near the last commanded angle
// first -- the angle can't jump far in one 50ms tick, so this saves
// a lot of predictApogee() calls (each with its own inner while-loop)
// on the Nano's software-float ATmega328P. Falls back to a full
// sweep only if the local search can't find a satisfying angle
// (e.g. right after launch, or a big/fast pressure change).
// ============================================================
float testAngle(int k, float alt, float vel) {
  float totalCdArea = rocketCd * rocketArea + 3.0 * getFinCd(k) * getFinArea(k);
  return predictApogee(alt, vel, totalCdArea);
}

float chooseFinAngle(float alt, float vel, float &predictedApogeeOut) {
  const int SEARCH_RADIUS = 2;   // +/- 2 table steps around last angle
  int lo = max(0, lastAngleIndex - SEARCH_RADIUS);
  int hi = min(NUM_ANGLES - 1, lastAngleIndex + SEARCH_RADIUS);

  for (int k = lo; k <= hi; k++) {
    float predApogee = testAngle(k, alt, vel);
    if (predApogee <= targetApogee_m) {
      predictedApogeeOut = predApogee;
      lastAngleIndex = k;
      return getFinAngle(k);
    }
  }

  // Local search didn't find a satisfying angle -- fall back to full sweep
  float targetFinAngle = 0;
  float predApogee = alt;
  for (int k = 0; k < NUM_ANGLES; k++) {
    predApogee = testAngle(k, alt, vel);
    if (predApogee <= targetApogee_m) {
      targetFinAngle = getFinAngle(k);
      predictedApogeeOut = predApogee;
      lastAngleIndex = k;
      return targetFinAngle;
    }
    if (k == NUM_ANGLES - 1) {
      targetFinAngle = getFinAngle(k);
      predictedApogeeOut = predApogee;
      lastAngleIndex = k;
    }
  }
  return targetFinAngle;
}

// ============================================================
// SD LOGGING
// ============================================================
void logHeader() {
  if (logFile) {
    logFile.println(F("time_s,rawAltitude_m,smoothedAltitude_m,velocity_mps,acceleration_mps2,deployed,targetAngle_deg,actualAngle_deg,predictedApogee_m,deployTime_s"));
    logFile.flush();
  }
}

void logRow(float t, float rawAlt, float alt, float vel, float acc, bool deployed, float targetAngle, float actualAngle, float predApogee, float deployTime) {
  if (logFile) {
    logFile.print(t, 3);            logFile.print(",");
    logFile.print(rawAlt, 3);       logFile.print(",");
    logFile.print(alt, 3);          logFile.print(",");
    logFile.print(vel, 3);          logFile.print(",");
    logFile.print(acc, 3);          logFile.print(",");
    logFile.print(deployed ? 1 : 0);logFile.print(",");
    logFile.print(targetAngle, 1);  logFile.print(",");
    logFile.print(actualAngle, 1);  logFile.print(",");
    logFile.print(predApogee, 2);   logFile.print(",");
    logFile.println(deployTime, 3);
    logFile.flush();   // flush every row so data survives a hard power loss at apogee/landing
  }
}

// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  while (!Serial);

  // ---- EEPROM read (last flight data) ----
  float lastApogee;
  bool lastFins;
  EEPROM.get(EEPROM_APOGEE_ADDR, lastApogee);
  EEPROM.get(EEPROM_FINS_ADDR, lastFins);
  Serial.print(F("Last flight apogee: ")); Serial.println(lastApogee);
  Serial.print(F("Fins deployed last flight: ")); Serial.println(lastFins ? F("YES") : F("NO"));

  // ---- BMP init ----
  if (!bmp.begin(0x76)) {
    Serial.println(F("BMP280 not found!"));
    while (1);
  }
  Serial.println(F("BMP280 connected!"));

  // ---- SD init ----
  if (!SD.begin(chipSelect)) {
    Serial.println(F("SD card init failed! Logging disabled."));
  } else {
    Serial.println(F("SD card ready."));
    // Create a fresh log file each flight; increment name if one exists
    char fname[16];
    int fileIndex = 0;
    do {
      snprintf(fname, sizeof(fname), "FLT%03d.CSV", fileIndex);
      fileIndex++;
    } while (SD.exists(fname) && fileIndex < 1000);

    logFile = SD.open(fname, FILE_WRITE);
    if (logFile) {
      Serial.print(F("Logging to: ")); Serial.println(fname);
      logHeader();
    } else {
      Serial.println(F("Failed to open log file!"));
    }
  }

  // ---- Servo init: sweep through EVERY preset angle and back, fast ----
  // Confirms the airbrake can actually reach each of the 10 table positions
  // (not just full open/close) before flight.
  airbrakeServo.attach(servoPin);
  startupServoSweep();

  // ---- Baseline altitude / target ----
  startingAlt = bmp.readAltitude(1013.25);   // meters
  targetApogee_m = startingAlt + targetApogeeAGL_m;
  prevAlt = startingAlt;

  // Pre-fill the smoothing buffer so the moving average doesn't ramp up from 0
  for (int i = 0; i < ALT_BUFFER_SIZE; i++) altBuffer[i] = startingAlt;
  altSum = startingAlt * ALT_BUFFER_SIZE;

  Serial.print(F("Starting altitude (m): ")); Serial.println(startingAlt);
  Serial.print(F("Target apogee (m AGL, absolute): ")); Serial.println(targetApogee_m);
}

// ============================================================
// MAIN LOOP
// ============================================================
void loop() {
  static float missionTime = 0;   // seconds since boot, used as the log time column

  // --- Read altitude (meters) and smooth it BEFORE differentiating ---
  float rawAlt = bmp.readAltitude(1013.25);
  float cAlt = smoothAltitude(rawAlt);   // ~1 ft BMP280 jitter filtered out here

  // --- Velocity & acceleration via slopes (now computed from smoothed altitude) ---
  float dAlt = cAlt - prevAlt;
  float vel_raw = dAlt / dt;
  float vel = 0.7 * prevVel + 0.3 * vel_raw;   // smoothing, same as original
  float acc = (vel - prevVel) / dt;

  prevAlt = cAlt;
  prevVel = vel;

  // --- Launch detection ---
  if (!launched) {
    if (vel > 15) {
      launchCounter++;
    } else {
      launchCounter = 0;
    }
    if (launchCounter >= 3) {
      launched = true;
      Serial.println(F("LAUNCH DETECTED"));
    }
  }

  // --- Flight logic ---
  float targetFinAngle = 0;
  float predApogee = cAlt;
  bool deployedThisStep = false;
  static float lastTargetAngle = -1;   // to detect COMMAND changes -> deploy-start time
  float deployTimeThisStep = -1;       // logs the moment a new angle is first commanded

  if (launched) {
    if (cAlt > cMaxAlt) cMaxAlt = cAlt;

    // Only during coast phase (matches MATLAB controller logic)
    if (vel > 0 && acc < 0) {
      // Cheap dead-reckon forward by the servo's lag time (no loop --
      // just kinematics) so the controller commands the angle it will
      // NEED once the servo actually gets there, not the angle needed
      // right now. This is the "lead time" compensation for servo lag.
      float projectedAlt = cAlt + vel * SERVO_LAG_S + 0.5 * acc * SERVO_LAG_S * SERVO_LAG_S;
      float projectedVel = vel + acc * SERVO_LAG_S;
      if (projectedVel < 0) projectedVel = 0;   // don't project past apogee

      targetFinAngle = chooseFinAngle(projectedAlt, projectedVel, predApogee);

      if (targetFinAngle > 0) {
        finsDeployed = true;
        deployedThisStep = true;
      }

      if (targetFinAngle != lastTargetAngle) {
        deployTimeThisStep = missionTime;   // command just changed -- servo starts moving now
        lastTargetAngle = targetFinAngle;
      }
    } else {
      targetFinAngle = 0;
      if (lastTargetAngle != 0) {
        deployTimeThisStep = missionTime;
        lastTargetAngle = 0;
      }
    }

    // Physically move the servo toward targetFinAngle, respecting speed/torque lag
    actuateServo(targetFinAngle, vel);
  }

  // --- Log every step: time, altitude, velocity, acceleration, deploy status,
  //     commanded angle, ACTUAL physical angle (accounts for ~0.5s servo lag),
  //     predicted apogee, and the moment each new deployment command was issued ---
  logRow(missionTime, rawAlt, cAlt, vel, acc, deployedThisStep, targetFinAngle, currentFinAngle, predApogee, deployTimeThisStep);

  // --- Apogee detection ---
  if (vel < 0) negativeVelCount++;
  else negativeVelCount = 0;

  if (negativeVelCount >= 3 && launched) {
    Serial.println(F("Max Altitude reached:"));
    Serial.println(cMaxAlt);

    EEPROM.put(EEPROM_APOGEE_ADDR, cMaxAlt);
    EEPROM.put(EEPROM_FINS_ADDR, finsDeployed);

    airbrakeServo.write(0);

    if (logFile) {
      logFile.flush();
      logFile.close();
    }

    while (1);
  }

  missionTime += dt;
  delay(dt * 1000);
}