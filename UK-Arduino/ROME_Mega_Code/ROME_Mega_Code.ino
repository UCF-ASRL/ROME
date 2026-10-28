// Include the ROME motor driver library
#include "ROME.h"
 
#define PI 3.14159265
#define WHEEL_DIAMETER 0.0762    // Diameter in meters of 6" VEX Pro Omniwheels 217-6200
#define PULSES_PER_REVOLUTION 12 // Motor specification
#define GEAR_RATIO 50            // 50:1 gear reduction
#define MAX_RPM 120              // Max RPM at 255 PWM signal
#define DT_PID 0.050             // PID loop execution period (seconds)
#define WATCHDOG_TIMEOUT_MS 250  // Comm watchdog timeout (ms)
 
// ---- ROME Motor Hardware Mapping ----
// Class instantiation: ROME(enablePin, encoderPin, pwmPin)
// Interrupt Pins: 2 (M1), 3 (M2), 20 (M3), 21 (M4)
ROME motor1(6,  2,  9);
ROME motor2(5,  3, 14);
ROME motor3(7, 20, 15);
ROME motor4(8, 21, 12);
ROME motors[] = {motor1, motor2, motor3, motor4};
 
// ---- Encoder & Speed Tracking ----
volatile unsigned long lastPulseTime[4] = {0, 0, 0, 0};
volatile int pulseCount[4] = {0, 0, 0, 0};
volatile float actualRPMs[4] = {0.0, 0.0, 0.0, 0.0};
 
int motorDirection[4] = {1, 1, 1, 1}; // 1 = Forward, -1 = Reverse
float desiredRPMs[4] = {0.0, 0.0, 0.0, 0.0};
float correctRPMs[4] = {0.0, 0.0, 0.0, 0.0};
int previousDirection[4] = {0, 0, 0, 0};
 
// ---- PI Gains & Anti-Windup ----
static double integralError[4] = {0, 0, 0, 0};
float Kp = 0.190;
float Ki = 0.125;
const float INTEGRAL_MAX = 500.0; // Dynamic anti-windup clamp limit
 
// ---- State Machine & Timing Flags ----
bool started = false;
static unsigned long lastPIDtime = 0;
uint32_t lastCommandTime = 0;
 
// ---- Serial Communications Buffer ----
static char lineBuf[96];
int lineLen = 0;
 
// ---- Function Declarations ----
void processSerialCommands();
void processLine(char* line);
void checkWatchdog();
void applyPID(const float rpm[4]);
void applyWheelRPM(const float rpm[4]);
void setMotorDirection(ROME &motor, int &previousDir, float rpmVal);
void setMotorSpeedRPM(ROME &motor, float rpmVal);
void calculateSpeed(int motorIndex);
void sendTelemetry();
 
// Interrupt Service Routines for Encoders
void updateSpeed1() { pulseCount[0]++; }
void updateSpeed2() { pulseCount[1]++; }
void updateSpeed3() { pulseCount[2]++; }
void updateSpeed4() { pulseCount[3]++; }
 
// ============================================================================
// SETUP & MAIN LOOP
// ============================================================================
void setup() {
  Serial1.begin(115200);
 
  // Initialize all motor channels to a disabled, safe state
  for (int i = 0; i < 4; i++) {
    motors[i].disableMotor();
    previousDirection[i] = 0;
  }
 
  // Configure Hardware Encoder Interrupt Pins
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  pinMode(20, INPUT_PULLUP);
  pinMode(21, INPUT_PULLUP);
 
  // Attach Hardware Interrupts
  attachInterrupt(digitalPinToInterrupt(2),  updateSpeed1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3),  updateSpeed2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(20), updateSpeed3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(21), updateSpeed4, CHANGE);
 
  lastCommandTime = millis();
}
 
void loop() {
  processSerialCommands();
  checkWatchdog();
 
  // Control loop executed at fixed DT_PID interval (20 Hz)
  if (millis() - lastPIDtime >= (DT_PID * 1000)) {
    for (int i = 0; i < 4; i++) {
      calculateSpeed(i);
    }
 
    if (started) {
      applyPID(desiredRPMs);
      applyWheelRPM(correctRPMs);
    } else {
      // Force immediate zero state when unstarted
      float zeros[4] = {0.0, 0.0, 0.0, 0.0};
      applyWheelRPM(zeros);
    }
 
    lastPIDtime = millis();
  }
 
  sendTelemetry();
}
 
// ============================================================================
// SERIAL INTERFACE & COMMAND PARSER
// ============================================================================
void processSerialCommands() {
  while (Serial1.available()) {
    char c = Serial1.read();
 
    if (c == '\r') continue;
 
    if (c == '\n') {
      lineBuf[lineLen] = '\0';
      processLine(lineBuf);
      lineLen = 0;
    } else if (lineLen < (int)sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = c;
    }
  }
}
 
void processLine(char* line) {
  // Command 1: System Handshake
  if (strcmp(line, "START_GV") == 0) {
    started = true;
    lastCommandTime = millis();
    return;
  }
 
  // Command 2: Shutdown / E-Stop (Fix #1: Clear 'started' state lock)
  if (strcmp(line, "STOP_ALL") == 0 || strcmp(line, "!") == 0) {
    started = false; // System lock-out; requires explicit START_GV to re-enable
    for (int i = 0; i < 4; i++) {
      desiredRPMs[i] = 0.0f;
      correctRPMs[i] = 0.0f;
      integralError[i] = 0.0f;
    }
    float zeros[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    applyWheelRPM(zeros);
    lastCommandTime = millis();
    return;
  }
 
  // Command 3: Continuous Speed Command Packet ("w1,w2,w3,w4")
  if (started) {
    float vals[4];
    int i = 0;
    char* token = strtok(line, ",");
 
    while (token != NULL && i < 4) {
      vals[i] = atof(token);
      i++;
      token = strtok(NULL, ",");
    }
 
    if (i == 4) {
      for (i = 0; i < 4; i++) {
        desiredRPMs[i] = vals[i];
      }
      lastCommandTime = millis();
    }
  }
}
 
// ============================================================================
// WATCHDOG & TELEMETRY
// ============================================================================
void checkWatchdog() {
  if (started) {
    if (millis() - lastCommandTime > WATCHDOG_TIMEOUT_MS) {
      // Fix #2: Zero both RPM targets and integral state during comm loss
      for (int i = 0; i < 4; i++) {
        desiredRPMs[i] = 0.0f;
        integralError[i] = 0.0f;
      }
    }
  }
}
 
void sendTelemetry() {
  static uint32_t lastTx = 0;
  if (millis() - lastTx < 50) return; // 20 Hz output rate limit
  lastTx = millis();
 
  // Format: "GV,act1,act2,act3,act4,cmd1,cmd2,cmd3,cmd4"
  Serial1.print("GV,");
  for (int i = 0; i < 4; i++) {
    Serial1.print(actualRPMs[i], 1);
    Serial1.print(",");
  }
  for (int i = 0; i < 4; i++) {
    Serial1.print(desiredRPMs[i], 1);
    if (i < 3) Serial1.print(",");
  }
  Serial1.println();
}
 
// ============================================================================
// CLOSED-LOOP PID & MOTOR DRIVE CONTROL
// ============================================================================
void applyPID(const float rpm[4]) {
  for (int i = 0; i < 4; i++) {
    double error = rpm[i] - actualRPMs[i];
    integralError[i] += error * DT_PID;
 
    // Integral windup protection
    integralError[i] = constrain(integralError[i], -INTEGRAL_MAX, INTEGRAL_MAX);
 
    double deltaSpeed = Kp * error + Ki * integralError[i];
    correctRPMs[i] = rpm[i] + deltaSpeed;
  }
}
 
void applyWheelRPM(const float rpm[4]) {
  for (int i = 0; i < 4; i++) {
    int currDir = (rpm[i] >= 0.0f) ? 1 : -1;
    motorDirection[i] = currDir;
 
    if (previousDirection[i] != 0 && previousDirection[i] != currDir) {
      motors[i].disableMotor();
    }
    setMotorDirection(motors[i], previousDirection[i], rpm[i]);
    setMotorSpeedRPM(motors[i], rpm[i]);
  }
}
 
void setMotorDirection(ROME &motor, int &previousDir, float rpmVal) {
  int currDir = (rpmVal >= 0.0f) ? 1 : -1;
  if (currDir > 0) motor.motorForward();
  else             motor.motorReverse();
  previousDir = currDir;
}
 
void setMotorSpeedRPM(ROME &motor, float rpmVal) {
  // High-precision float mapping to [0, 255] PWM output
  float magnitude = fabs(rpmVal);
  int pwm = (int)((magnitude / (float)MAX_RPM) * 255.0f);
  pwm = constrain(pwm, 0, 255);
 
  motor.motorOn(pwm);
}
 
void calculateSpeed(int motorIndex)
{
    unsigned long currentTime = micros();
    unsigned long timeDifference =
        currentTime - lastPulseTime[motorIndex];

    if(timeDifference >= (DT_PID * 1000000UL))
    {
        int counts;

        noInterrupts();

        counts = pulseCount[motorIndex];
        pulseCount[motorIndex] = 0;
        lastPulseTime[motorIndex] = currentTime;

        interrupts();

        double timeInSeconds =
            timeDifference / 1000000.0;

        double pulsesPerMinute =
            (counts / timeInSeconds) * 60.0;

        double rpm =
            pulsesPerMinute /
            (PULSES_PER_REVOLUTION * GEAR_RATIO);

        actualRPMs[motorIndex] =
            motorDirection[motorIndex] * rpm;
    }
}