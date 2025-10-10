#include <Stepper.h>
#include <AccelStepper.h>
#include <Encoder.h>
#include <imxrt.h>

// Limits of each joint, degrees from zero. Expressed in Motor frame.
// J2,J3,and J5 reversed here.
//Old limits?? Unsure why these were picked
//float limits[6] = {-170.0,132.0,-141.0,-155.0,105.0,-155.0};
//Zero set straight up
float limits[6] = {-180.0,42.5,-145.0,-155.0,86.5,155.0};


// Expressed in Joint frame
long steps[6] = {0,0,0,0,0,0}; 
float stepsSec[6] = {0.0,0.0,0.0,0.0,0.0,0.0};

// Steps per degree for each motor
float stepsDeg[6] = {1/(.022368421*2),1/.018082192,
                     1/.017834395,1/.021710526,
                     1/.045901639,1/.046792453};
// Assign pin numbers to stepper
byte pinNumbers[6][2] = {{0,1},{2,3},{4,5},{6,7},{8,9},{10,11}};

//Initialize encoder object
Encoder encoders[] = {
  Encoder(14,15),
  Encoder(16,17),
  Encoder(18,19),
  Encoder(20,21),
  Encoder(22,23),
  Encoder(24,25)
};


class LimitSwitch{
  public:
    int pin;
    //state of the switch. 0 = open, 1 = closed
    int state;

};
//Time for debouncing limit switches
float debounceTime = 100;
float lastDebounce[6] = {0,0,0,0,0,0};
LimitSwitch LS[6];

AccelStepper steppers[] = {AccelStepper(AccelStepper::DRIVER, pinNumbers[0][0], pinNumbers[0][1]),
                           AccelStepper(AccelStepper::DRIVER, pinNumbers[1][0], pinNumbers[1][1]),
                           AccelStepper(AccelStepper::DRIVER, pinNumbers[2][0], pinNumbers[2][1]),
                           AccelStepper(AccelStepper::DRIVER, pinNumbers[3][0], pinNumbers[3][1]),
                           AccelStepper(AccelStepper::DRIVER, pinNumbers[4][0], pinNumbers[4][1]),
                           AccelStepper(AccelStepper::DRIVER, pinNumbers[5][0], pinNumbers[5][1])};

float loopTime = 0;

void setup() {
  //Set drive strength for stepper 4
  //pinMode(6, OUTPUT);
  //pinMode(7, OUTPUT);
  IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_10 =
    IOMUXC_PAD_DSE(2) |
    IOMUXC_PAD_SPEED(2) |
    IOMUXC_PAD_SRE; 
  IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_11 =
    IOMUXC_PAD_DSE(2) |
    IOMUXC_PAD_SPEED(2) |
    IOMUXC_PAD_SRE;
  //digitalWrite(6, HIGH);
  //digitalWrite(7, HIGH);
  //Limit switch setup
  for (int i = 0; i<6; i++) {
    //Signal pins on teensy
    LS[i].pin = 26+i;
    //Pulls signal pin up
    pinMode(LS[i].pin, INPUT_PULLUP);
  }
  // Motor Setup
  int maxSpeed = 1000;
  
  for (int i = 0; i < 6; i++)
  {
    steppers[i].setMaxSpeed(maxSpeed);
    steppers[i].setSpeed(0);
    steppers[i].setMinPulseWidth(5);
    steppers[i].enableOutputs();
    if(i == 3){
      steppers[3].setMinPulseWidth(10);
    }
  }
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  for(int i = 0;i < 6;i++)
      {
        //if(i == 3) continue;
        // Check for reversed action
        if ((i == 0) || (i==2)) {//|| (i == 2)) { //((i == 1) || (i == 2)) || (i == 5){
          
          steppers[i].setSpeed(-500);
          //Move each motor to limit switch(with debounce) and then set limits.
          Serial.print("Running motor to limit:");
          Serial.println(i);
          while (true) {
            steppers[i].runSpeed();
            if ((digitalRead(LS[i].pin) == LOW) && (millis() - lastDebounce[i] > debounceTime)) {
              lastDebounce[i] = millis();
              break;
            }
          }
          Serial.println("Limit found. Setting limit now.");
          // Set current position in Motor frame 
          steppers[i].setCurrentPosition((long)(limits[i]*stepsDeg[i]));
          encoders[i].write((long)(limits[i]*stepsDeg[i]));
          // If reversed, must transform to Joint frame
          steps[i] = -(long)(limits[i]*stepsDeg[i]);
          steppers[i].moveTo(0);
          steppers[i].setMaxSpeed(1000);
          steppers[i].setAcceleration(500); // Smooth acceleration
          steppers[i].runToPosition();
        }
        else{
          steppers[i].setSpeed(500);
          //Move each motor to limit switch and then set limits.
          Serial.print("Running motor to limit:");
          Serial.println(i);
          while (true) {
            steppers[i].runSpeed();
            if ((digitalRead(LS[i].pin) == LOW) && (millis() - lastDebounce[i] > debounceTime)) {
              lastDebounce[i] = millis();
              break;
            }
          }
          Serial.println("Limit found. Setting limit now.");
          // Set current position in Motor frame 
          steppers[i].setCurrentPosition((long)(limits[i]*stepsDeg[i]));
          encoders[i].write((long)(limits[i]*stepsDeg[i]));
          // If reversed, must transform to Joint frame
          steps[i] = (long)(limits[i]*stepsDeg[i]);
          steppers[i].moveTo(0);
          steppers[i].setMaxSpeed(1000);
          steppers[i].setAcceleration(500); // Smooth acceleration
          steppers[i].runToPosition();
        }
        stepsSec[i] = 0.0;
      
      }
}
