#include <Stepper.h>
#include <AccelStepper.h>
#include <PacketSerial.h>
#include <Encoder.h>
#include <imxrt.h>

#define STEPPER_CREATE 0x01
#define SET_RADSEC 0x02
#define UPDATE_STEPPERS 0x03
#define SET_STATES 0x04
#define UPDATE_STATES 0x05
#define CALIBRATE 0x06
#define READ 0x07
#define SET 0x08
#define READ_ALL 0x09

PacketSerial packetizer;

//Initialize encoder object
Encoder encoders[] = {
  Encoder(14,15),
  Encoder(17,16),
  Encoder(18,19),
  Encoder(20,21),
  Encoder(22,23),
  Encoder(24,25)
};

// Number of steppers
int NUM = 6;
int posMode = 0;

// Expressed in Joint frame
long steps[6] = {0,0,0,0,0,0}; 
float stepsSec[6] = {0.0,0.0,0.0,0.0,0.0,0.0};

// Steps per Deg for each motor
float stepsDeg[6] = {1/(.022368421*2),1/.018082192,
                     1/.017834395,1/.021710526,
                     1/.045901639,1/.046792453};
//Uses ar3 motor values
/*
float stepsDeg[6] = {1/(.018),1/.036,
                     1/.036,1/.021710526,
                     1/1.8,1/1.8};
*/
//Ar2 motor values
//float stepsDeg[6] = {44.70833342, 0, 0, 0, 0, 0};
//float gearRat[6] = {.082840775, 1, 1, 1, 1, 1}; //from motor to output, multiply. from output to motor, divide
float d2r = 3.14159/180;
// Limits of each joint, degrees from zero. Expressed in Motor frame.
// J2,J3,and J5 reversed here. Old Limits
//float limits[6] = {-170.0,132.0,-141.0,-155.0,105.0,-155.0};
//float limits[6] = {-170, 0, 0, 155, 105, 155};
float limits[6] = {-170, 0, 1, -164, 104, 148};
//Limits for arm pointing up
//float limits[6] = {-170.0,42.5,-144.0,-155.0,86.5,155.0};
//float limits[6] = {-180.0,42.5,-145.0,-155.0,86.5,155.0};

// Assign pin numbers to stepper
byte pinNumbers[6][2] = {{0,1},{2,3},{4,5},{6,7},{8,9},{10,11}};

class LimitSwitch{
  public:
    int pin;
    //state of the switch. 0 = open, 1 = closed
    int state;

};
//Time for debouncing limit switches
const float debounceTime = 100;
float lastDebounce[6] = {0,0,0,0,0,0};
LimitSwitch LS[6];

AccelStepper steppers[] = {AccelStepper(AccelStepper::DRIVER, pinNumbers[0][0], pinNumbers[0][1]),
                           AccelStepper(AccelStepper::DRIVER, pinNumbers[1][0], pinNumbers[1][1]),
                           AccelStepper(AccelStepper::DRIVER, pinNumbers[2][0], pinNumbers[2][1]),
                           AccelStepper(AccelStepper::DRIVER, pinNumbers[3][0], pinNumbers[3][1]),
                           AccelStepper(AccelStepper::DRIVER, pinNumbers[4][0], pinNumbers[4][1]),
                           AccelStepper(AccelStepper::DRIVER, pinNumbers[5][0], pinNumbers[5][1])};


void setup() 
{
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

  // Packetizer Setup
  packetizer.begin(19200);
  packetizer.setPacketHandler(&onPacketReceived);
}

void loop() 
{
  packetizer.update();
  updateSteppers();

}

void onPacketReceived(const uint8_t* buffer, size_t size)
{
  // Get command ID, which is a single byte
  uint8_t CMD = buffer[0];

  switch(CMD)
  {
    case SET_STATES:
    {
      byte ID = buffer[1];
      posMode = 1;

      byte targetPos[4];
      byte targetVelocity[4];

      for (int i = 0; i < 4;i++)
      {
        targetPos[i] = buffer[i+2];
        targetVelocity[i] = buffer[i+2+4];
      }

      // Cast 4-byte array to long 
      steps[ID] = *((long*)targetPos);
      // Cast 4-byte array to float
      stepsSec[ID] = *((float*)targetVelocity);
      break;
    }
    
    case UPDATE_STATES:
    {
      posMode = 1;
      
      byte targetPositions[6][4];
      byte targetVelocities[6][4];

      // Read byte data from buffer
      for (int i = 0;i < 6;i++)
      {
        for (int j = 0; j < 4;j++)
        {
          int index = i*4 + j + 1;
          targetPositions[i][j] = buffer[index];
          targetVelocities[i][j] = buffer[index + 24];
        }
      }

      // Update current position and velocity commands
      for (int i = 0;i < 6;i++)
      {
        // Cast 4-byte array to long 
        steps[i] = *((long*)targetPositions[i]);
        // Cast 4-byte array to float
        stepsSec[i] = *((float*)targetVelocities[i]);
      }

      break;
    }
    case READ:
    {
      byte ID = buffer[1];
      //int32_t count = steppers[ID].currentPosition();
      //Attempt with encoders:
      int32_t count = encoders[ID].read();
      byte result[4];
      
      result[0] = (count & 0x000000ff);
      result[1] = (count & 0x0000ff00) >> 8;
      result[2] = (count & 0x00ff0000) >> 16;
      result[3] = (count & 0xff000000) >> 24;

      packetizer.send(result,sizeof(result));
      break;
    }
    case READ_ALL:
    {
      int32_t position[6];
      byte result[4*NUM];

      for (int i = 0;i < NUM;i++)
      {
        //position[i] = steppers[i].currentPosition();
        //Attempt with encoders:
        position[i] = encoders[i].read();
        result[4*i] = (position[i] & 0x000000ff);
        result[4*i+1] = (position[i] & 0x0000ff00) >> 8;
        result[4*i+2] = (position[i] & 0x00ff0000) >> 16;
        result[4*i+3] = (position[i] & 0xff000000) >> 24;
      }

      packetizer.send(result,sizeof(result));
      break;
    }
    case CALIBRATE:
    { 
      /*
      for(int i = 2;i < 6;i++)
      {
        // Check for reversed action
        if ((i == 1) || (i == 2) || (i == 4) || (i == 5)) {

          steppers[i].setSpeed(-500);
          //Move each motor to limit switch and then set limits.
          while (digitalRead(LS[i].pin) != LOW) {
            steppers[i].runSpeed();
          }
          // Set current position in Motor frame 
          steppers[i].setCurrentPosition((long)(limits[i]*stepsDeg[i]));
          encoders[i].write((long)(limits[i]*stepsDeg[i]));
          // If reversed, must transform to Joint frame
          steps[i] = -(long)(limits[i]*stepsDeg[i]);
        }
        else {

          steppers[i].setSpeed(500);
          //Move each motor to limit switch and then set limits.
          while (digitalRead(LS[i].pin) != LOW) {
            steppers[i].runSpeed();
          }
          // Set current position in Motor frame 
          steppers[i].setCurrentPosition((long)(limits[i]*stepsDeg[i]));
          encoders[i].write((long)(limits[i]*stepsDeg[i]));
          // If reversed, must transform to Joint frame
          steps[i] = (long)(limits[i]*stepsDeg[i]);
        }
        stepsSec[i] = 0.0;
      
      }*/
      calibrate();
      break;
    
    }
    default:
    {
      // do nothing
      break;
    }
  }
}

void updateSteppers()
{
  int reversed;

  if (posMode == 1)
  {
    // Update steppers
    for (int i = 0; i < 6; i++)
    {
      if ((i==0) || (i==2))//((i == 1) || (i == 2) || (i == 4) || (i == 5))
        reversed = -1;
      else
        reversed = 1;
      
      //steppers[i].moveTo(reversed * steps[i]);
      //steppers[i].setSpeed(-stepsSec[i]);
      //steppers[i].runSpeedToPosition();
      steppers[i].moveTo(reversed * steps[i]);
      steppers[i].setMaxSpeed(abs(stepsSec[i]));
      steppers[i].setAcceleration(500); // Smooth acceleration
      steppers[i].run();
    
    }
  }
  else
  {
    for (int i = 0;i < 6;i++)
      //steppers[i].runSpeed();
      steppers[i].run();
  }
}

void updateSteppersToleranced()
{
  int reversed;
  const float Kp = .5;
  const int maxError = 10; // Max steps before intervention
  const int maxCorrectionSpeed = 1000; //Max speed to correct
  if (posMode == 1)
  {
    // Update steppers
    for (int i = 0; i < 6; i++)
    {
      if ((i==0)) //|| (i==2))//((i == 1) || (i == 2) || (i == 4) || (i == 5))
        reversed = -1;
      else
        reversed = 1;
      //Setup proportional error correction 
      long target = reversed*steps[i];
      long actual = encoders[i].read();
      long error = target - actual;

      if (abs(error)>maxError) {
        float correction = Kp*error;
        float totalspeed = correction + stepsSec[i];    
      }
      steppers[i].moveTo(reversed * steps[i]);
      steppers[i].setSpeed(-stepsSec[i]);
      steppers[i].runSpeedToPosition();
    }
  }
  else
  {
    for (int i = 0;i < 6;i++)
      //steppers[i].runSpeed();
      steppers[i].run();
  }
}

void calibrate() {
  for(int i = 0;i < 6;i++){
      
        if(i == 3) continue;
        // Check for reversed action
        if ((i == 0)  || (i==2)){//|| (i == 2)) { //((i == 1) || (i == 2)) || (i == 5){
          
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
          steppers[i].moveTo(-135*stepsDeg[i]);
          if (i==2) {
            steppers[i].moveTo(120*stepsDeg[i]);
          }
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
          if (i==1) {
            //steppers[i].moveTo(42.5*stepsDeg[i]);
          }
          steppers[i].setMaxSpeed(1000);
          steppers[i].setAcceleration(500); // Smooth acceleration
          steppers[i].runToPosition();
        }
        stepsSec[i] = 0.0;
      
      }
}
//transforms desired output angles of robot to input angles of motors using gear ratios and steps/deg
//ex: if you want the base to rotate 90 degrees(theta_out), multiply by stepDeg to get phi_out(step_out)
//then divide by the gear ratio to get the number of steps the motor must turn to make that happen(step_mot)  h
void accelTransform(int theta_out[5], int stepDeg[5], int gearRat[5]){
  for(int i=0;i<6;i++){
    //step_mot = theta_out[i]*stepDeg[i]/gearRat[i];
  }
}

