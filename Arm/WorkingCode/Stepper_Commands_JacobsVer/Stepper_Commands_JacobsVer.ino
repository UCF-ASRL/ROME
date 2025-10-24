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
#define encTol .11 //tolerance encoderRunToVal must reach i.e error must be less than encTol steps
#define encOffset .12 //offset from encoder readings to cw/ccwtest was around .12

//Selected motor speeds
int maxSpeed = 1500;
int calspeed = 1500;
int maxAccel = 500;
int runSpeed = 1500;
//Calculated encoder count per motor step
float countstep[6] = {20.48, 20.48, 20.48, 20.48, 20.48, 20.48}; 

PacketSerial packetizer;

//Initialize encoder object
Encoder encoders[] = {
  Encoder(14,15),
  Encoder(17,16),
  Encoder(18,19),
  Encoder(1000,1000),
  Encoder(21,20),
  Encoder(22,23)
};

// Number of steppers
int NUM = 6;
int posMode = 0;

// Expressed in Joint frame
long steps[6] = {0,0,0,0,0,0}; 
float stepsSec[6] = {0.0,0.0,0.0,0.0,0.0,0.0};

//Ar2 motor values
//Steps input to angle output(includes mechanical ratios, microstepping etc.)
//See Step Angle Troubleshooting excel file for calculations
float stepsDeg[6] = {44.44444444, 55.55555556, 55.55555556, 46.66666667, 10, 20};
float limits[6] = {-180, 132, 141, -165, 90, 180}; //Limits in direction of limit switch(NOT ccw or cw limits... see step angle troubleshooting sheet);
float otherLimits[6] = {160, 0, 1, 165, -90, -170}; //Non limit-switched limits 
//holds negative rotation values for some motors => Motor direction depends on wiring order of A+ A-, B+ and B-. 
//Only change if wiring from stepper to stepper driver changes.
int negspeeds[6] = {1, -1, 1, 1, -1, -1};
//Arbitrary starting position.
int startpos[6] = {0, 90, 2, 1, 0, 0};
// Assign pin numbers to stepper
byte pinNumbers[6][2] = {{0,1},{2,3},{4,5},{6,7},{8,9},{10,11}};
//False if motor is uncalibrated, True if calibrated
bool calibrated[6] = {false, false, false, false, false, false};

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
{/*
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
      
      encoderRunToVal(i,steps[i],)
    } 

    }
  else
  {
    for (int i = 0;i < 6;i++)
      //steppers[i].runSpeed();
      steppers[i].run();
  }
    */
  for (int i=0;i<6;i++){
    bool targetReached = false;
    int path = ValidateTraj(i,steps[i]/stepsDeg[i]);
      if (path == 2) {
        continue;
      }

    encoderRunToVal(i, steps[i], path);

    if(abs(encoders[i].read()/countstep[i]/stepsDeg[i]-steps[i]/stepsDeg[i])<encTol*2) {
      targetReached = true;
      Serial.println("Target Reached");
    }
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
      
    if(i == 3 || i == 4) continue;
      // Check for reversed action
      steppers[i].setSpeed(-calspeed*negspeeds[i]);
      //Move each motor to limit switch(with debounce) and then set limits.
      Serial.print("Running motor to limit:");
      Serial.println(limits[i]);
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
      encoders[i].write((long)(limits[i]*stepsDeg[i]*countstep[i]));
      // If reversed, must transform to Joint frame
      steps[i] = -(long)(limits[i]*stepsDeg[i]);
      encoderRunToVal(i, startpos[i]*stepsDeg[i], 3);
      /*if(abs(encoders[i].read()/countstep[i]/stepsDeg[i]-startpos[i])<encTol*2) {
        Serial.print("Output angle is ");
        Serial.print(encoders[i].read()/countstep[i]/stepsDeg[i]);
        Serial.print("degrees or "); 
        Serial.print(encoders[i].read()/countstep[i]);
        Serial.println("steps.");
        calibrated[i] = true;
        Serial.println("Starting position achieved");
      } */
    }
}

void encoderRunToVal(int x, float targetSteps, int path) {
  //float encTol = .01;
  while(true) {
    long encoderVal = encoders[x].read();
     //Converts encoder reading from counts to steps.
    long actual = encoderVal/countstep[x];// In steps
    long error = targetSteps - actual; // In steps
      
    //error variable holds the location data and corrects for any incorrections from the motor
    //if error < limit the position has been reached
    // if error > limit the motor will continue to move to the desired location which is current plus error
    if(abs(error) < encTol){
      break;
    }
        
    steppers[x].setAcceleration(maxAccel);
    if(x!=2){
      if (path == 3){
        steppers[x].setSpeed(runSpeed*negspeeds[x]);
      } else if (path == 0) {
        steppers[x].setSpeed(-runSpeed*negspeeds[x]);//-
      } else if (path == 1) {
        steppers[x].setSpeed(runSpeed*negspeeds[x]);//+
      } else {
        return;
      }
    } else if(x == 2){
        if (path == 3){
          steppers[x].setSpeed(runSpeed*negspeeds[x]);
      } else if (path == 0) {
          steppers[x].setSpeed(runSpeed*negspeeds[x]);//-
      } else if (path == 1) {
          steppers[x].setSpeed(-runSpeed*negspeeds[x]);//+
      } else {
        return;
      }
    }
    steppers[x].moveTo(encoders[x].read()/countstep[x] + error);
    steppers[x].run();
    //Serial.println(encoders[x].read());
  }
}

int ValidateTraj(int x, float deg){
  //Change resolution of trajectory
  float step = encTol; //Can find a path within .1 deg
  float tol = 2*step; //Atleast 2*step

  //Ensure input is within limits
  if ((deg < limits[x] && deg > otherLimits[x]) || (deg > limits[x] && deg < otherLimits[x])){
    
  } else {
    Serial.println("Input is out of range. Please select another value.");
    return 2;
  }
  //so limits arent overwritten:
  int intermediate = 0;
  int vlimit = limits[x];
  int votherLimit = otherLimits[x];
  //flip limits so following path works for all motors 
  /*
  if (negspeeds[x] < 0) { 
    intermediate = vlimit;
    vlimit = votherLimit;
    votherLimit = intermediate;
  }
  */
  //Ensure path stays within limits
  float cwtest = encoders[x].read()/countstep[x]/stepsDeg[x]; //Consistently getting an offset of encoder values from previous reading to cw/ccwtest
  float ccwtest = encoders[x].read()/countstep[x]/stepsDeg[x]; //encOffset should fix
  //If path = 0, take counterclockwise path, if path = 1, take clockwise path, if path = 2 Don't move
  int path = 2;
  //Serial.println(cwtest);
  //Serial.println(ccwtest);
  /*
  if ((abs(cwtest-deg) < encTol) || (abs(ccwtest-deg) < encTol)){
    Serial.println("Angle is already within tolerance of target");
    return 2;
  }
  */
  while ((betweenPoints(vlimit, votherLimit, cwtest) == true) || (betweenPoints(vlimit, votherLimit, ccwtest) == true)){
      if (betweenPoints(vlimit,votherLimit,cwtest) == false) { //(cwtest>votherLimit || cwtest<vlimit)
        Serial.println("Clockwise path is out of bounds");
        //Serial.println(cwtest);
        path = 2; 
        cwtest = 1000; //traps cwtest from reentering valid range 
        
      }
      if (betweenPoints(vlimit,votherLimit,ccwtest) == false) { //(ccwtest>votherLimit || ccwtest<vlimit)
        Serial.println("Counterclockwise path is out of bounds");
        path = 2; 
        ccwtest = 1000; //traps ccwtest from reentering valid range 
        
      }
      if ((abs(cwtest - deg) < tol) && (betweenPoints(vlimit, deg, cwtest) == false)) {
        /*
        Serial.println(vlimit);
        Serial.println(deg);
        Serial.println(cwtest);
        Serial.println(abs(cwtest - deg));
        Serial.println(betweenPoints(vlimit, deg, cwtest));
        */
        Serial.println("Clockwise path is invalid!");
        path = 2; //No path valid yet.
      } else if ((abs(cwtest - deg) < tol) && (betweenPoints(vlimit, deg, cwtest) == true)) {
        Serial.println("Clockwise path is valid!");
        path = 1; //go clockwise path
        break;
      } 
      if ((abs(ccwtest - deg) < tol) && (betweenPoints(votherLimit, deg, ccwtest) == false)) {
        /*
        Serial.println(votherLimit);
        Serial.println(deg);
        Serial.println(ccwtest);
        Serial.println(abs(ccwtest - deg));
        Serial.println(betweenPoints(votherLimit, deg, ccwtest));
        */
        Serial.println("Counterclockwise path is invalid!");
        path = 2; //No valid path yet.
      } else if ((abs(ccwtest - deg) < tol) && (betweenPoints(votherLimit, deg, ccwtest) == true)) {
        Serial.println("Counterclockwise path is valid!");
        path = 0; //go counterclockwise path
        break;
      }
      if (negspeeds[x] < 0){
        cwtest = cwtest-step;
        ccwtest = ccwtest+step;
        //Serial.print("CW: ");
        //Serial.print(cwtest);
        //Serial.print(" CCW: ");
        //Serial.println(ccwtest);
      } else {
        cwtest = cwtest+step;
        ccwtest = ccwtest-step;
      }
    }
    //Since we flipped limits before must flip direction again to keep consistent.
    /*if (negspeeds[x] < 0) { 
      if(path == 0) {
        path = 1;
      } else if(path == 1) {
        path = 0;
      } else {
        path = 2;
      }
      
    }
    */
    return path;
}
bool betweenPoints(float limit, float input, float testpoint) {
  /*limit = fmod((limit + 360),360);
  input = fmod((input + 360),360);
  testpoint = fmod((testpoint + 360),360);
  */
  if(limit < input) {
    if(testpoint<input && testpoint>limit) {
      //Serial.println("true1");
      return true;
    } else {
        //Serial.println("false1");
        return false;
    }
  } else if(limit > input) {
      if(testpoint>input && testpoint<limit){
        //Serial.println("true2");
        return true;
      } else {
          //Serial.println("false2");
          return false;
      }
  } else {
    //Serial.println("false3");
    return false;
  }
}
