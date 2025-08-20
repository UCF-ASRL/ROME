#include <AccelStepper.h>
#include <Encoder.h>
// Define only one motor in this test, but use slot [5] for stepper 6
#define TEST_STEPPER_INDEX 5

// Create a 2D array for all 6 steppers, even if we only use one
int pinNumbers[6][2] = {
  {0, 1},    // Stepper 1
  {2, 3},    // Stepper 2
  {4, 5},    // Stepper 3
  {6, 7},    // Stepper 4
  {8, 9},    // Stepper 5
  {10, 11}   // Stepper 6 (index 5) <-- CHANGE TO YOUR WIRING
};
//Define 5 Encoders
//NOTE: Stepper 4 does not have an encoder. So Encoder 4 and 5 are paired with steppers 5 and 6
Encoder encoders[5];


// Define 6 steppers
AccelStepper steppers[] = {
  AccelStepper(AccelStepper::DRIVER, pinNumbers[0][0], pinNumbers[0][1]),
  AccelStepper(AccelStepper::DRIVER, pinNumbers[1][0], pinNumbers[1][1]),
  AccelStepper(AccelStepper::DRIVER, pinNumbers[2][0], pinNumbers[2][1]),
  AccelStepper(AccelStepper::DRIVER, pinNumbers[3][0], pinNumbers[3][1]),
  AccelStepper(AccelStepper::DRIVER, pinNumbers[4][0], pinNumbers[4][1]),
  AccelStepper(AccelStepper::DRIVER, pinNumbers[5][0], pinNumbers[5][1])
};
int lG = 0;
int hG = 360/.046792453;
void setup() {
  Serial.begin(115200);
  //steppers[TEST_STEPPER_INDEX].setPinsInverted(false, true, true);
  // Set STEP pin for stepper 6 (index 5) HIGH (idle)
  //setup encoders
  for (int i=0; i<5;i++) {
    encoders[i].pin1 = 2*i + 14;  
    encoders[i].pin2 = 2*i + 15;
  }
  /*
  for (int i=0;i<6;i++) {
    pinMode(pinNumbers[i][0], OUTPUT);
    pinMode(pinNumbers[i][1], OUTPUT);
    digitalWrite(pinNumbers[i][1], HIGH);  // Initial DIR LOW or HIGH depending on need
    digitalWrite(pinNumbers[i][0], HIGH);
  

  }
  */
  steppers[TEST_STEPPER_INDEX].setMinPulseWidth(5);
  steppers[TEST_STEPPER_INDEX].enableOutputs();
  steppers[TEST_STEPPER_INDEX].setMaxSpeed(2000);
  steppers[TEST_STEPPER_INDEX].setAcceleration(1000);
  //steppers[TEST_STEPPER_INDEX].setCurrentPosition(0);
  steppers[TEST_STEPPER_INDEX].moveTo(50000);
}

void loop() {
  if(steppers[TEST_STEPPER_INDEX].currentPosition() == lG) {
    Serial.println("Current position is: 0");
    Serial.print("Going to ");
    Serial.println(hG);
    steppers[TEST_STEPPER_INDEX].moveTo(hG);
    steppers[TEST_STEPPER_INDEX].run();
    Serial.println("Turned");
  } else if(steppers[TEST_STEPPER_INDEX].currentPosition() == hG) {
    Serial.print("Going to ");
    Serial.println(lG);
    steppers[TEST_STEPPER_INDEX].moveTo(lG);
    steppers[TEST_STEPPER_INDEX].run();
    Serial.println("Turned");
  } 
  while(steppers[TEST_STEPPER_INDEX].currentPosition() != lG && steppers[TEST_STEPPER_INDEX].currentPosition() != hG) {//if(steppers[TEST_STEPPER_INDEX].currentPosition() != 0 && steppers[TEST_STEPPER_INDEX].currentPosition() != 1000){
    Serial.print("Current Position is: ");
    Serial.println(steppers[TEST_STEPPER_INDEX].currentPosition());
    steppers[TEST_STEPPER_INDEX].run();
    Serial.println("Turned");
  }
  
  
  
/*
  int val = digitalRead(pinNumbers[TEST_STEPPER_INDEX][0]);
  Serial.print("Stepper 6 STEP pin (pin ");
  Serial.print(pinNumbers[TEST_STEPPER_INDEX][0]);
  Serial.print("): ");
  Serial.println(val == HIGH ? "HIGH" : "LOW");

  delay(200);*/
}

void lowlevel(){
  
}