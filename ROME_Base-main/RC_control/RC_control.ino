#include <ROME.h>

ROME motor1(5,22,9);  
ROME motor2(6,3,10); 
ROME motor3(7,18,11);
ROME motor4(8,2,12);

#define RCPinFR 21 // pin for Forward and Reverse channel 2
#define RCPinLR 20 // pin for Left and Right channel 1
#define RCPinSLSR 19 // pin for swivel left and swivel right channel 4

// initializing for Forward and reverse movement 
int pulseWidthFR = 0;
int PwmValFR = 0;
int PwmSigFR = 0;
volatile long startTimeFR = 0;
volatile long currrentTimeFR = 0;
volatile long pulsesFR = 0;

// initializing for Left and Right
int pulseWidthLR = 0;
int PwmValLR = 0;
int PwmSigLR = 0;
volatile long startTimeLR = 0;
volatile long currrentTimeLR = 0;
volatile long pulsesLR = 0;

// initializing for swivel left & right 
int pulseWidthSLSR = 0;
int PwmValSLSR = 0;
int PwmSigSLSR = 0;
volatile long startTimeSLSR = 0;
volatile long currrentTimeSLSR = 0;
volatile long pulsesSLSR = 0;

void setup()
{
  Serial.begin(9600); 
  pinMode(RCPinFR, INPUT_PULLUP); 
  pinMode(RCPinLR, INPUT_PULLUP);
  pinMode(RCPinSLSR, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RCPinFR), pulseTimerFR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RCPinLR), pulseTimerLR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RCPinSLSR), pulseTimerSLSR, CHANGE);
}

int getJoystickPosition(int PwmValFR, int PwmValLR, int PwmValSLSR) {
  int deadband = 15;  // Tolerance for joystick centering

  if (PwmValFR > 15 && abs(PwmValLR) <= deadband) return 1;        // Forward
  else if (PwmValFR < -15 && abs(PwmValLR) <= deadband) return 2;  // Reverse
  else if (abs(PwmValFR) <= deadband && PwmValLR > 15) return 3;   // Right
  else if (abs(PwmValFR) <= deadband && PwmValLR < -15) return 4;  // Left
  else if (PwmValFR > 15 && PwmValLR > 15) return 5;   // Forward Right
  else if (PwmValFR > 15 && PwmValLR < -15) return 6;  // Forward Left
  else if (PwmValFR < -15 && PwmValLR > 15) return 7;  // Swivel Right
  else if (PwmValFR < -15 && PwmValLR < -15) return 8; // Swivel Left
  else return 0;                                       // Stop
}

void loop()
{
  if (pulsesFR < 2000 && pulsesLR < 2000 && pulsesSLSR < 2000)
  {
    pulseWidthFR = pulsesFR;
    PwmValFR = map(pulseWidthFR, 985, 1995, -255, 255);

    pulseWidthLR = pulsesLR;
    PwmValLR = map(pulseWidthLR, 985, 1995, -255, 255);

    pulseWidthSLSR = pulsesSLSR;
    PwmValSLSR = map(pulseWidthSLSR, 985, 1995, -255, 255);

    int position = getJoystickPosition(PwmValFR, PwmValLR, PwmSigSLSR);
    switch (position) {
      case 1: // Forward
      PwmSigFR = map(PwmValFR, 15, 255, 0 , 255); // map 
      motor1.motorReverse();
      motor2.motorForward();
      motor3.motorForward();
      motor4.motorReverse();
      delay(400);
      motor1.motorOn(PwmSigFR);
      motor2.motorOn(PwmSigFR);
      motor3.motorOn(PwmSigFR);
      motor4.motorOn(PwmSigFR);
      break;

    case 2: // Reverse
      PwmSigFR = map(PwmValFR, -15, -255, 0 , 255);
      motor1.motorForward();
      motor2.motorReverse();
      motor3.motorReverse();
      motor4.motorForward();
      delay(400);
      motor1.motorOn(PwmSigFR);
      motor2.motorOn(PwmSigFR);
      motor3.motorOn(PwmSigFR);
      motor4.motorOn(PwmSigFR);
      break;

    case 3: // Right
      PwmSigLR = map(PwmValLR, 15, 255, 0 , 255);
      motor1.motorForward();
      motor2.motorForward();
      motor3.motorReverse();
      motor4.motorReverse();
      delay(400);
      motor1.motorOn(PwmSigLR);
      motor2.motorOn(PwmSigLR);
      motor3.motorOn(PwmSigLR);
      motor4.motorOn(PwmSigLR);
      break;

    case 4: // Left
      PwmSigLR = map(PwmValLR, -15, -255, 0 , 255);
      motor1.motorReverse();
      motor2.motorReverse();
      motor3.motorForward();
      motor4.motorForward();
      delay(400);
      motor1.motorOn(PwmSigLR);
      motor2.motorOn(PwmSigLR);
      motor3.motorOn(PwmSigLR);
      motor4.motorOn(PwmSigLR);
      break;


      case 5: // Forward Right
      PwmSigFR = map(PwmValFR, 15, 255, 0, 255);
      PwmSigLR = map(PwmValLR, 15, 255, 0, 255);
      motor1.disableMotor();
      motor2.motorForward();
      motor3.disableMotor();
      motor4.motorReverse();
      delay(400);
      motor1.disableMotor();
      motor2.motorOn(PwmSigFR);
      motor3.disableMotor();
      motor4.motorOn(PwmSigFR);
      break;

    case 6: // Forward Left
      PwmSigFR = map(PwmValFR, 15, 255, 0, 255);
      PwmSigLR = map(PwmValLR, -15, -255, 0, 255);
      motor1.motorReverse();
      motor2.disableMotor();
      motor3.motorForward();
      motor4.disableMotor();
      delay(400);
      motor1.motorOn(PwmSigFR);
      motor2.disableMotor();
      motor3.motorOn(PwmSigFR);
      motor4.disableMotor();
      break;

    case 7: // Swivel Right
      PwmSigFR = map(PwmValFR, -15, -255, 0, 255);
      PwmSigLR = map(PwmValLR, 15, 255, 0, 255);
      motor1.motorForward();
      motor2.disableMotor();
      motor3.motorForward();
      motor4.disableMotor();
      delay(400);
      motor1.motorOn(PwmSigFR);
      motor2.disableMotor();
      motor3.motorOn(PwmSigFR);
      motor4.disableMotor();
      break;

    case 8: // Swivel Left
      PwmSigFR = map(PwmValFR, -15, -255, 0, 255);
      PwmSigLR = map(PwmValLR, -15, -255, 0, 255);
      motor1.disableMotor();
      motor2.motorReverse();
      motor3.disableMotor();
      motor4.motorReverse();
      delay(400);
      motor1.disableMotor();
      motor2.motorOn(PwmSigFR);
      motor3.disableMotor();
      motor4.motorOn(PwmSigFR);
      break;

      default:
        delay(100); 
        motor1.disableMotor(); 
        motor2.disableMotor();
        motor3.disableMotor();
        motor4.disableMotor();
        delay(100);
        break;
    }
  }

  // Serial.print(PwmValFR);
  // Serial.print("    ");
  // Serial.print(PwmValLR);
  // Serial.print("    ");
  Serial.print(PwmValSLSR);
  Serial.print("    ");
}

void pulseTimerFR()   // for Forward or Reverse interrupt 
{
  currrentTimeFR = micros();
    if (currrentTimeFR > startTimeFR)
    {
     pulsesFR = currrentTimeFR - startTimeFR;
     startTimeFR = currrentTimeFR;
    }
}

void pulseTimerLR()   // for Left or Right interrupt 
{
  currrentTimeLR = micros();
    if (currrentTimeLR > startTimeLR)
    {
     pulsesLR = currrentTimeLR - startTimeLR;
     startTimeLR = currrentTimeLR;
    }
}

void pulseTimerSLSR()   // for swivel Left or Right interrupt 
{
  currrentTimeSLSR = micros();
    if (currrentTimeSLSR > startTimeSLSR)
    {
     pulsesSLSR = currrentTimeSLSR - startTimeSLSR;
     startTimeSLSR = currrentTimeSLSR;
    }
}