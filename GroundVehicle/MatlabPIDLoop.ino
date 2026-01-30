// Include the ROME library.
#include <ROME.h> 
#define PI 3.14159265
#define WHEEL_DIAMETER 0.0762  // Diameter in meters of 6" VEX Pro Omniwheels 217-6200
#define PULSES_PER_REVOLUTION 12  //motor's specification
#define gearRatio 50 // 50:1 gear reduction with gear box attached
#define MAX_RPM 120 //actual max RPM when running suspended wheel at 255 pwm signal
#define DT_PID 0.050 //time in sec between each PID implementation

// ---- ROME motor wiring (unchanged) ----
ROME motor1(6,  2,  9);
ROME motor2(5,  3, 14);
ROME motor3(7, 20, 15);
ROME motor4(8, 21, 12);
ROME motors[] = {motor1, motor2, motor3, motor4};

// For Encoder Data
volatile unsigned long lastPulseTime[4] = {0, 0, 0, 0};
static unsigned long lastPIDtime = 0;
static unsigned long lastDebugTime = 0;
volatile int pulseCount[4] = {0, 0, 0, 0};
int motorDirection[4] = {1, 1, 1, 1}; // 1 for forward, -1 for reverse
float desiredRPMs[4] = {0.0, 0.0, 0.0, 0.0};
float correctRPMs[4] = {0.0, 0.0, 0.0, 0.0};
volatile float actualRPMs[4] = {0.0, 0.0, 0.0, 0.0};
bool runOver = 0;

//for PI loop
static double integralError[4] = {0,0,0,0};

float Kp = 0.190; //experimentally found using MotorRampUp code and Matlab tuning script
float Ki = 0.125;

// float Kp = 0.000; //no correction mode
// float Ki = 0.000;

// Previous directions: 1 = fwd, -1 = rev
int previousDirection[] = {0, 0, 0, 0};
bool started = false;

// Parsing buffer
static char lineBuf[96];
int lineLen = 0;

void setup() 
{
  // Serial.begin(115200); // optional debug
  Serial1.begin(115200);

  // Ensure motors initially off (manual per request)
  for (int i = 0; i < 4; i++) 
  {
    motors[i].disableMotor();
    previousDirection[i] = 0;
  }

    pinMode(2, INPUT_PULLUP);
    pinMode(3, INPUT_PULLUP);
    pinMode(20, INPUT_PULLUP);
    pinMode(21, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(2), updateSpeed1, CHANGE); //set encoder interrupts
  attachInterrupt(digitalPinToInterrupt(3), updateSpeed2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(20), updateSpeed3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(21), updateSpeed4, CHANGE);
}

void loop() 
{
  // Wait for single 'Y' to start. This is sent from Matlab terminal by Bluetooth when path is ran.
  if (!started) 
  {
    if (Serial1.available()) 
    {
      char c = Serial1.read();
      // Serial.println(c);   // show startup handshake
      if (c == 'Y') {
        started = true;

        delay(100);
      }
    }
    return;
  }

  if(millis() - lastPIDtime >= (DT_PID * 1000))
  {
    calculateSpeed(0); calculateSpeed(1); calculateSpeed(2); calculateSpeed(3);
    applyPID(desiredRPMs);
    applyWheelRPM(correctRPMs);

    lastPIDtime = millis();
  }

  // Accumulate chars until newline; then parse and execute
  while (Serial1.available()) 
  {
    char c = Serial1.read();

    if (c == '\r') 
    {
      continue;
    }

    if (lineLen < (int)sizeof(lineBuf) - 1) 
    {
      lineBuf[lineLen++] = c;
    }

    if (c == '\n') 
    {
    lineBuf[lineLen] = '\0';
    processLine(lineBuf);
    lineLen = 0;
    }
  }

//if (millis() - lastDebugTime >= DT_DEBUG && started)  // prints out desired vs actual
// {
//   Serial.print("M1: Desired: ");
//   Serial.print(desiredRPMs[0], 2);
//   Serial.print(" Actual: ");
//   Serial.print(actualRPMs[0], 2);
//   Serial.print("    ");

//   Serial.print("M2: Desired: ");
//   Serial.print(desiredRPMs[1], 2);
//   Serial.print(" Actual: ");
//   Serial.print(actualRPMs[1], 2);
//   Serial.print("    ");

//   Serial.print("M3: Desired: ");
//   Serial.print(desiredRPMs[2], 2);
//   Serial.print(" Actual: ");
//   Serial.print(actualRPMs[2], 2);
//   Serial.print("    ");

//   Serial.print("M4: Desired: ");
//   Serial.print(desiredRPMs[3], 2);
//   Serial.print(" Actual: ");
//   Serial.println(actualRPMs[3], 2); // println to end the line

//   lastDebugTime = millis();
// }

}

void processLine(char* line) 
{
  // //debug
  // Serial.print("RAW LINE: ");
  // Serial.print(line);

  float vals[4];
  int i = 0;
  char* token = strtok(line, ","); //getting individual float vals

  if(token && strcmp(token, "!") == 0)
  {
      started = false;
      return;
  }

  while (token != NULL && i < 4) 
  {
    vals[i] = atof(token);   // convert string → float
    i++;
    token = strtok(NULL, ",");
  }

  // Serial.print("Tokens parsed = ");
  // Serial.println(i);

  if (i == 4) 
  {
    for(i = 0; i < 4; i++)
    {
      desiredRPMs[i] = vals[i];
    }
  }
}

void applyPID(const float rpm[4]) 
{

    for(int i = 0; i < 4; i++)
    {
      double error = rpm[i] - actualRPMs[i];
      integralError[i] += error * DT_PID;
      double deltaSpeed = Kp * error + Ki * integralError[i];
      correctRPMs[i] = rpm[i] + deltaSpeed;
    }
}


void applyWheelRPM(const float rpm[4]) 
{
  // If direction changes, briefly disable before reversing
  for (int i = 0; i < 4; i++) 
  {
    int currDir = (rpm[i] >= 0.0f) ? 1 : -1;
    motorDirection[i] = currDir;
    if (previousDirection[i] != 0 && previousDirection[i] != currDir) 
    {
      motors[i].disableMotor();
    }
  }

  // Set directions
  for (int i = 0; i < 4; i++) 
  {
    setMotorDirection(motors[i], previousDirection[i], rpm[i]);
  }

  // Set speeds
  for (int i = 0; i < 4; i++) 
  {
    setMotorSpeedRPM(motors[i], rpm[i]);
  }
}

void setMotorDirection(ROME &motor, int &previousDir, float rpmVal) 
{
  int currDir = (rpmVal >= 0.0f) ? 1 : -1;

  if (currDir > 0) motor.motorForward();
  else             motor.motorReverse();

  previousDir = currDir;
}

void setMotorSpeedRPM(ROME &motor, float rpmVal)
{
  // Map ±MAX_RPM to ±255 PWM
  int pwm = map(rpmVal, -MAX_RPM, MAX_RPM, -255, 255);
  motor.motorOn(abs(pwm));
}

//register new pulse on each ISR call
void updateSpeed1() 
{ 
  pulseCount[0]++;
}
void updateSpeed2() 
{ 
  pulseCount[1]++;
}
void updateSpeed3() 
{ 
  pulseCount[2]++;
}
void updateSpeed4()     
{ 
  pulseCount[3]++;
}

void calculateSpeed(int motorIndex)
{
  unsigned long currentTime = micros();
  unsigned long timeDifference = currentTime - lastPulseTime[motorIndex];
  if(timeDifference >= (DT_PID * 1000000UL))
  {
    double timeInSeconds = timeDifference / 1000000.0; // Convert microseconds to seconds

    // Calculate pulses per minute
    double pulsesPerMinute = (pulseCount[motorIndex] / timeInSeconds) * 60;

    // Calculate RPM
    double rpm = pulsesPerMinute / (PULSES_PER_REVOLUTION * gearRatio);
    actualRPMs[motorIndex] = motorDirection[motorIndex] * rpm;

    // Reset pulse count after calculations
    noInterrupts();
    pulseCount[motorIndex] = 0;
    lastPulseTime[motorIndex] = currentTime;  // Update the last pulse time for future calculations 
    interrupts(); 
  }
}
