#include <Arduino.h>
#include <Stepper.h>
#include <AccelStepper.h>
#include <Encoder.h>
#include <imxrt.h>

// ============================================================================
// CONFIGURATION & CONSTANTS
// ============================================================================
#define encTol 0.11f      // Tolerance in steps
#define encOffset 0.12f   // Offset from encoder readings

// Watchdog timeout in milliseconds
const uint32_t WATCHDOG_TIMEOUT_MS = 500;

// Speeds and Accelerations
int maxSpeed = 2000;
int calspeed[6] = {1200, 800, 800, 600, 800, 800};
int maxAccel = 500;
int runSpeed = 750;

// Encoder count per motor step
float countstep[6] = {20.48, 20.48, 20.48, 20.48, 20.48, 20.48};

// Steps per degree conversion factors
float stepsDeg[6] = {44.44444444, 55.55555556, 55.55555556, 46.66666667, 10.0, 20.0};

// Joint angle limits (Degrees)
float limits[6]      = {-180.0, 132.0, 141.0, -165.0,  90.0,  180.0}; // Limit-switch side
float otherLimits[6] = { 160.0,   0.0,   1.0,  165.0, -90.0, -170.0}; // Non-limit side

// Direction multipliers depending on wiring logic
int negspeeds[6] = {-1, 1, 1, 1, 1, -1};

// Dynamic Home Position (updated via HOME command)
float homePosDeg[6] = {0.0, 90.0, 90.0, 1.0, 0.0, 0.0};

// Pin definitions
byte pinNumbers[6][2] = {{0,1}, {2,3}, {4,5}, {6,7}, {8,9}, {10,11}};

// ============================================================================
// STATE MACHINE & TARGET CONTROL
// ============================================================================
enum ArmState {
  UNCALIBRATED,
  CALIBRATING,
  READY,
  FAULT,
  LIMIT_TEST  // Fixed: Added comma on previous line
};

ArmState armState = UNCALIBRATED;

enum HomeState { IDLE, MOVING, DONE };
HomeState homeState[6] = {IDLE, IDLE, IDLE, IDLE, IDLE, IDLE};

float desiredJointDeg[6] = {0.0, 90.0, 90.0, 1.0, 0.0, 0.0};
float actualJointDeg[6]  = {0.0, 0.0,  0.0,  0.0, 0.0, 0.0};

uint32_t lastCommandTime = 0;

// ============================================================================
// HARDWARE OBJECTS
// ============================================================================
// Joint 4 lacks physical encoder hardware. (1000, 1000) safely disables ISR attachment.
Encoder encoders[] = {
  Encoder(14, 15),
  Encoder(17, 16), // Reversed encoder direction alignment
  Encoder(18, 19),
  Encoder(1000, 1000), // Joint 4 safe library-recognized placeholder
  Encoder(21, 20),
  Encoder(22, 23)
};

struct LimitSwitch {
  int pin;
  int state;
};

const float debounceTime = 100.0;
float lastDebounce[6] = {0, 0, 0, 0, 0, 0};
LimitSwitch LS[6];

AccelStepper steppers[] = {
  AccelStepper(AccelStepper::DRIVER, pinNumbers[0][0], pinNumbers[0][1]),
  AccelStepper(AccelStepper::DRIVER, pinNumbers[1][0], pinNumbers[1][1]),
  AccelStepper(AccelStepper::DRIVER, pinNumbers[2][0], pinNumbers[2][1]),
  AccelStepper(AccelStepper::DRIVER, pinNumbers[3][0], pinNumbers[3][1]),
  AccelStepper(AccelStepper::DRIVER, pinNumbers[4][0], pinNumbers[4][1]),
  AccelStepper(AccelStepper::DRIVER, pinNumbers[5][0], pinNumbers[5][1])
};

// ============================================================================
// FUNCTION DECLARATIONS
// ============================================================================
void processSerialCommands();
void updateROMEArm();
void updateUncalibratedJog();
void updateTelemetry();
void checkWatchdog();
void fullCalibrate();
void encoderRunToVal_nb(int x, float targetSteps, int path);
int ValidateTraj(int x, float deg);
bool betweenPoints(float limit, float input, float testpoint);

// ============================================================================
// SETUP & LOOP
// ============================================================================
void setup() {
  Serial.begin(115200);
  // Initialize Serial6 for ESP32 communications (RX6 = Pin 25, TX6 = Pin 24)
  Serial6.begin(115200);

  // Configure drive strength for Joint 4 pins
  IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_10 = IOMUXC_PAD_DSE(2) | IOMUXC_PAD_SPEED(2) | IOMUXC_PAD_SRE;
  IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_11 = IOMUXC_PAD_DSE(2) | IOMUXC_PAD_SPEED(2) | IOMUXC_PAD_SRE;

  // Setup Limit Switches
  for (int i = 0; i < 6; i++) {
    LS[i].pin = 26 + i;
    pinMode(LS[i].pin, INPUT_PULLUP);
  }

  // Setup Steppers
  for (int i = 0; i < 6; i++) {
    steppers[i].setMaxSpeed(maxSpeed);
    steppers[i].setSpeed(0);
    steppers[i].setMinPulseWidth(5);
    steppers[i].enableOutputs();
    if (i == 3) {
      steppers[3].setMinPulseWidth(10);
    }
  }

  lastCommandTime = millis();
}

void loop() {
  processSerialCommands();
  checkWatchdog();
  updateROMEArm();
  updateUncalibratedJog(); // Added JOG Engine
  updateTelemetry();
}

// ============================================================================
// SERIAL COMMUNICATIONS & COMMAND PARSER
// ============================================================================
void processSerialCommands() {
  static String inputBuffer = "";

  while (Serial6.available() > 0) {
    char c = (char)Serial6.read();

    if (c == '\n' || c == '\r') {
      inputBuffer.trim();
      if (inputBuffer.length() > 0) {
        
        // Command 1: CAL_ARM
        if (inputBuffer == "CAL_ARM") {
          armState = CALIBRATING;
          fullCalibrate();
          armState = READY;
          Serial6.println("ARM_READY");
          lastCommandTime = millis();
        }
        
        // Command 2: TEST_LIMITS (Hardware Diagnostics)
        else if (inputBuffer == "TEST_LIMITS") {
          armState = LIMIT_TEST;
          for (int i = 0; i < 6; i++) {
            steppers[i].setSpeed(0);
            steppers[i].move(0);
            desiredJointDeg[i] = actualJointDeg[i];
          }
          Serial6.println("LIMIT_TEST_READY");
          lastCommandTime = millis();
        }
        
        // Command 3: STOP_ALL (E-Stop / Motion Hold / Test Exit)
        else if (inputBuffer == "STOP_ALL") {
          if (armState == LIMIT_TEST) armState = UNCALIBRATED;
          for (int i = 0; i < 6; i++) {
            steppers[i].move(0); // Safely halt uncalibrated jogs
            desiredJointDeg[i] = actualJointDeg[i];
          }
          lastCommandTime = millis();
        }
        
        // Command 4: STATUS Query
        else if (inputBuffer == "STATUS") {
          switch (armState) {
            case UNCALIBRATED: Serial6.println("UNCALIBRATED"); break;
            case CALIBRATING:  Serial6.println("CALIBRATING");  break;
            case READY:        Serial6.println("READY");        break;
            case FAULT:        Serial6.println("FAULT");        break;
            case LIMIT_TEST:   Serial6.println("LIMIT_TEST");   break;
          }
          lastCommandTime = millis(); // Refresh watchdog during long jogs
        }
        
        // Command 5: HOME,d1,d2,d3,d4,d5,d6
        else if (inputBuffer.startsWith("HOME,")) {
          int idx = 5;
          for (int i = 0; i < 6; i++) {
            int nextIdx = inputBuffer.indexOf(',', idx);
            if (nextIdx == -1 && i == 5) nextIdx = inputBuffer.length();
            if (nextIdx != -1) {
              float parsedVal = inputBuffer.substring(idx, nextIdx).toFloat();
              homePosDeg[i] = parsedVal;
              desiredJointDeg[i] = parsedVal;
              idx = nextIdx + 1;
            }
          }
          lastCommandTime = millis();
        }
        
        // Command 6: JOG,joint,delta (Uncalibrated manual jogging)
        else if (inputBuffer.startsWith("JOG,")) {
          int firstComma = inputBuffer.indexOf(',');
          int secondComma = inputBuffer.indexOf(',', firstComma + 1);
          if (firstComma != -1 && secondComma != -1) {
            int j = inputBuffer.substring(firstComma + 1, secondComma).toInt();
            float delta = inputBuffer.substring(secondComma + 1).toFloat();
            
            // Only allow manual jogging if NOT in normal tracking state
            if (j >= 0 && j < 6 && armState != READY) {
              int toward = (otherLimits[j] > limits[j]) ? 1 : -1;
              long stepTarget = delta * stepsDeg[j] * negspeeds[j] * toward;
              
              steppers[j].setMaxSpeed(runSpeed);
              steppers[j].setAcceleration(maxAccel);
              steppers[j].move(stepTarget);
              
              lastCommandTime = millis();
            }
          }
        }
        
        // Command 7: d1,d2,d3,d4,d5,d6 (Continuous Angle Target Command)
        else if (armState == READY && !inputBuffer.startsWith("JOG")) {
          float parsedDeg[6];
          int count = 0;
          int idx = 0;

          while (idx < (int)inputBuffer.length() && count < 6) {
            int nextIdx = inputBuffer.indexOf(',', idx);
            if (nextIdx == -1) nextIdx = inputBuffer.length();
            parsedDeg[count++] = inputBuffer.substring(idx, nextIdx).toFloat();
            idx = nextIdx + 1;
          }

          if (count == 6) {
            for (int i = 0; i < 6; i++) desiredJointDeg[i] = parsedDeg[i];
            lastCommandTime = millis();
          }
        }
      }
      inputBuffer = "";
    } else {
      inputBuffer += c;
    }
  }
}

// ============================================================================
// WATCHDOG PROTECTION
// ============================================================================
void checkWatchdog() {
  if (millis() - lastCommandTime > WATCHDOG_TIMEOUT_MS) {
    if (armState == READY) {
      // Hold position: clamp target angles to actual positions
      for (int i = 0; i < 6; i++) {
        desiredJointDeg[i] = actualJointDeg[i];
      }
    } else {
      // Cancel any active uncalibrated jogs
      for (int i = 0; i < 6; i++) {
        steppers[i].move(0);
      }
    }
  }
}

// ============================================================================
// CONTINUOUS TARGET TRACKING
// ============================================================================
void updateROMEArm() {
  if (armState != READY) return;

  for (int i = 0; i < 6; i++) {
    if (i == 3) {
        float targetSteps = desiredJointDeg[3] * stepsDeg[3];
        steppers[3].moveTo((long)targetSteps);
        steppers[3].run();
        actualJointDeg[3] = desiredJointDeg[3];
        continue;
    }

    int path = ValidateTraj(i, desiredJointDeg[i]);
    if (path == 2) continue;

    float targetSteps = desiredJointDeg[i] * stepsDeg[i];
    encoderRunToVal_nb(i, targetSteps, path);
    actualJointDeg[i] = (float)encoders[i].read() / countstep[i] / stepsDeg[i];
  }
}

// ============================================================================
// UNCALIBRATED JOG ENGINE (Safe Relative Moves)
// ============================================================================
void updateUncalibratedJog() {
  if (armState == READY || armState == LIMIT_TEST) return;
  
  for (int i = 0; i < 6; i++) {
    if (steppers[i].distanceToGo() != 0) {
      
      // Safety Check: Prevent driving deeper into a pressed switch
      if (digitalRead(LS[i].pin) == LOW) {
        int moveDir = (steppers[i].distanceToGo() > 0) ? 1 : -1;
        int switchDir = (negspeeds[i] > 0) ? -1 : 1; 
        
        if (moveDir == switchDir) {
          steppers[i].move(0); // Hard stop! Driving into switch.
          continue; 
        }
      }
      steppers[i].run();
    }
  }
}

// ============================================================================
// TELEMETRY OUTPUT STREAM
// ============================================================================
void updateTelemetry() {
  static uint32_t lastTelemetryTime = 0;
  if (millis() - lastTelemetryTime >= 50) { // 20 Hz updates
    lastTelemetryTime = millis();

    // DIAGNOSTIC STREAM
    if (armState == LIMIT_TEST) {
      Serial6.print("LIMITS,");
      for (int i = 0; i < 6; i++) {
        int isPressed = (digitalRead(LS[i].pin) == LOW) ? 1 : 0;
        Serial6.print(isPressed);
        if (i < 5) Serial6.print(",");
      }
      Serial6.println();
      return; 
    }

    // NORMAL STREAM
    const char* stateStr = "UNCALIBRATED";
    switch (armState) {
      case CALIBRATING: stateStr = "CALIBRATING"; break;
      case READY:       stateStr = "READY";       break;
      case FAULT:       stateStr = "FAULT";       break;
      default:          stateStr = "UNCALIBRATED"; break;
    }

    Serial6.print("ARM,");
    for (int i = 0; i < 6; i++) {
      Serial6.print(actualJointDeg[i], 1);
      Serial6.print(",");
    }
    Serial6.println(stateStr);
  }
}

// ============================================================================
// HOMING & CALIBRATION PROCEDURES
// ============================================================================
void fullCalibrate() {
  bool calibration_finished = false;
  bool limit_hit[6] = {false, false, false, false, false, false};

  for (int i = 0; i < 6; i++) homeState[i] = IDLE;

  for (int i = 0; i < 6; i++) {
    steppers[i].setSpeed(-calspeed[i] * negspeeds[i]);
  }

  // PHASE 1: Drive ALL motors to switches
  while (!limit_hit[0] || !limit_hit[1] || !limit_hit[2] || !limit_hit[3] || !limit_hit[4] || !limit_hit[5]) {
    for (int i = 0; i < 6; i++) {
      if (limit_hit[i]) steppers[i].setSpeed(0);
      steppers[i].runSpeed();
    }

    for (int i = 0; i < 6; i++) {
      if (!limit_hit[i] && (digitalRead(LS[i].pin) == LOW) && (millis() - lastDebounce[i] > debounceTime)) {
        lastDebounce[i] = millis();
        steppers[i].setCurrentPosition((long)(limits[i] * stepsDeg[i]));

        if (i != 3) {
          encoders[i].write((long)(limits[i] * stepsDeg[i] * countstep[i]));
        }
        limit_hit[i] = true;

        // DEBUG: Notify MATLAB that limit was struck
        Serial6.print("LIMIT,");
        Serial6.println(i);
      }
    }
  }

  // PHASE 2A: Drive joints WITH encoders to home
  while (!calibration_finished) {
    calibration_finished = true;
    for (int i = 0; i < 6; i++) {
      if (i == 3) continue;
      if (homeState[i] != DONE) {
        encoderRunToVal_nb(i, homePosDeg[i] * stepsDeg[i], 3);
        if (homeState[i] != DONE) calibration_finished = false;
      }
    }
  }

  // PHASE 2B: Move Joint 4 open-loop home
  steppers[3].setMaxSpeed(runSpeed);
  steppers[3].setAcceleration(maxAccel);
  steppers[3].runToNewPosition((long)(homePosDeg[3] * stepsDeg[3]));
  homeState[3] = DONE;

  // PHASE 3: Sync variables
  for (int i = 0; i < 6; i++) {
    actualJointDeg[i]  = homePosDeg[i];
    desiredJointDeg[i] = homePosDeg[i];
  }
  steppers[3].setCurrentPosition((long)(homePosDeg[3] * stepsDeg[3]));
}

// ============================================================================
// KINEMATICS & TRAJECTORY UTILITIES
// ============================================================================
void encoderRunToVal_nb(int x, float targetSteps, int path) {
  long encoderVal = encoders[x].read();
  long actual = encoderVal / countstep[x];
  long error = targetSteps - actual;

  if (abs(error) < encTol) {
    steppers[x].setSpeed(0);
    homeState[x] = DONE;
    return;
  }

  steppers[x].setAcceleration(maxAccel);
  if (path != 0 && path != 1 && path != 3) return;
  int toward = (otherLimits[x] > limits[x]) ? 1 : -1;
  int dir    = (error > 0) ? 1 : -1;
  steppers[x].setSpeed(runSpeed * negspeeds[x] * toward * dir);

  steppers[x].moveTo(actual + error);
  steppers[x].run();
  homeState[x] = MOVING;
}

int ValidateTraj(int x, float deg) {
  float step = encTol;
  float tol = 2 * step;

  if (!((deg >= limits[x] && deg <= otherLimits[x]) || (deg <= limits[x] && deg >= otherLimits[x]))) {
    return 2;
  }

  int vlimit = limits[x];
  int votherLimit = otherLimits[x];

  float cwtest = encoders[x].read() / countstep[x] / stepsDeg[x];
  float ccwtest = encoders[x].read() / countstep[x] / stepsDeg[x];
  int path = 2;

  if ((abs(cwtest - deg) < encTol) || (abs(ccwtest - deg) < encTol)) {
    return 2;
  }

  while (betweenPoints(vlimit, votherLimit, cwtest) || betweenPoints(vlimit, votherLimit, ccwtest)) {
    if (!betweenPoints(vlimit, votherLimit, cwtest)) {
      path = 2;
      cwtest = 1000;
    }
    if (!betweenPoints(vlimit, votherLimit, ccwtest)) {
      path = 2;
      ccwtest = 1000;
    }
    if ((abs(cwtest - deg) < tol) && !betweenPoints(vlimit, deg, cwtest)) {
      path = 2;
    } else if ((abs(cwtest - deg) < tol) && betweenPoints(vlimit, deg, cwtest)) {
      path = 1;
      break;
    }
    if ((abs(ccwtest - deg) < tol) && !betweenPoints(votherLimit, deg, ccwtest)) {
      path = 2;
    } else if ((abs(ccwtest - deg) < tol) && betweenPoints(votherLimit, deg, ccwtest)) {
      path = 0;
      break;
    }

    if (negspeeds[x] < 0) {
      cwtest -= step;
      ccwtest += step;
    } else {
      cwtest += step;
      ccwtest -= step;
    }
  }
  return path;
}

bool betweenPoints(float limit, float input, float testpoint) {
  if (limit < input) {
    return (testpoint < input && testpoint > limit);
  } else if (limit > input) {
    return (testpoint > input && testpoint < limit);
  }
  return false;
}