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
//#define x 1 //Motor to be tested (Note: 0 --> motor 1, 1 --> motor 2 etc)


//Selected motor speeds
int maxSpeed = 1000;
int calspeed = 1000;
int maxAccel = 500;
int runSpeed = 1000;
//Calculated encoder count per motor step
float countstep[6] = {20.48, 20.48, 20.48, 20.48, 20.48, 20.48}; 
PacketSerial packetizer;

//Initialize encoder object
Encoder encoders[] = {
  Encoder(14,15),
  Encoder(17,16),//reversed because encoder direction does not align with motor direction
  Encoder(18,19),
  Encoder(0,0),
  Encoder(21,20),
  Encoder(22,23)
};

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
int startpos[6] = {0, 90, 0, 1, 0, 0};
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
  clearScreen();
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
}


void loop() {
  //Read serial for input
  menu();
  
}


void singleCalibrate(int x) {
  //Set speed and account for motor positive/negative direction(based on motor wiring)
  steppers[x].setSpeed(-calspeed*negspeeds[x]);
  Serial.print("running motor to limit:");
  Serial.println(limits[x]);
  //run until limit switch is triggered
  while(true) {
    steppers[x].runSpeed();
    //debounce function to avoid limit switch readings oscillating
    if ((digitalRead(LS[x].pin) == LOW) && (millis() - lastDebounce[x] > debounceTime)) {
      lastDebounce[x] = millis();
      break;
    }
  }
  Serial.println("Limit found. Setting limit now");
  //limit switch position is at known limit. update accelstepper and encoder position to this limit
  //MUST MULTIPLY BY stepsdeg and countstep RESPECTIVELY!!! LIMITS ARE IN DEG, ACCELSTEPPER IN STEPS, ENCODER IN COUNTS
  steppers[x].setCurrentPosition((long)(limits[x]*stepsDeg[x])); //deg * step/deg = step
  encoders[x].write((long)(limits[x]*stepsDeg[x]*countstep[x])); // deg * step/deg * count/step = count
  Serial.println(encoders[x].read());
  Serial.print("Moving to ");
  Serial.print(startpos[x]);
  Serial.print(" degrees or ");
  Serial.print(startpos[x]*stepsDeg[x]);
  Serial.println(" steps");
  encoderRunToVal(x, startpos[x]*stepsDeg[x], 3); //Use encoder feedback to move to starting position set at beginning of file
  if(abs(encoders[x].read()/countstep[x]/stepsDeg[x]-startpos[x])<encTol*2) {
    Serial.print("Output angle is ");
    Serial.print(encoders[x].read()/countstep[x]/stepsDeg[x]);
    Serial.print("degrees or "); 
    Serial.print(encoders[x].read()/countstep[x]);
    Serial.println("steps.");
    calibrated[x] = true;
    Serial.println("Starting position achieved");
  } 
}
//NOT FINISHED
/*void fullCalibrate() {
  //Set speed and account for motor positive/negative direction(based on motor wiring)
  steppers[x].setSpeed(-calspeed*negspeeds[x]);
  Serial.print("running motor to limit:");
  Serial.println(limits[x]);
  //run until limit switch is triggered
  while(true) {
    steppers[x].runSpeed();
    //debounce function to avoid limit switch readings oscillating
    if ((digitalRead(LS[x].pin) == LOW) && (millis() - lastDebounce[x] > debounceTime)) {
      lastDebounce[x] = millis();
      break;
    }
  }
  Serial.println("Limit found. Setting limit now");
  //limit switch position is at known limit. update accelstepper and encoder position to this limit
  //MUST MULTIPLY BY stepsdeg and countstep RESPECTIVELY!!! LIMITS ARE IN DEG, ACCELSTEPPER IN STEPS, ENCODER IN COUNTS
  steppers[x].setCurrentPosition((long)(limits[x]*stepsDeg[x])); //deg * step/deg = step
  encoders[x].write((long)(limits[x]*stepsDeg[x]*countstep[x])); // deg * step/deg * count/step = count
  Serial.println(encoders[x].read());
  Serial.print("Moving to ");
  Serial.print(startpos[x]);
  Serial.print(" degrees or ");
  Serial.print(startpos[x]*stepsDeg[x]);
  Serial.println(" steps");
  encoderRunToVal(startpos[x]*stepsDeg[x], 3); //Use encoder feedback to move to starting position set at beginning of file
  delay(50);
  Serial.print("Output angle is ");
  Serial.print(encoders[x].read()/countstep[x]/stepsDeg[x]);
  Serial.print("degrees or "); 
  Serial.print(encoders[x].read()/countstep[x]);
  Serial.println("steps.");
}
*/

//Function to validate trajectory. 
//Checks to see if input is within limit range
//Checks if CW or CCW path from current postion to target is valid(within the limits)
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

  if ((abs(cwtest-deg) < encTol) || (abs(ccwtest-deg) < encTol)){
    Serial.println("Angle is already within tolerance of target");
    return 2;
  }

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
        Serial.println(encoders[x].read());
      }
}
//Function that puts the robot in a safe power off position.
void fullPowerOff() {
  Serial.println("Moving to power off position");
  float powerOffpos[6] = {0, 132, 141, 0, 0, 0};
  bool withinTol[6] = {false, false, false, false, false, false};
  for (int i=1;i<6;i++) {
    steppers[i].moveTo(powerOffpos[i]);
    steppers[i].run();  
  }
  for(int i=1;i<6;i++){
    if(abs(encoders[i].read()/countstep[i]/stepsDeg[i]-powerOffpos[i])<encTol*2) {
      withinTol[i] = true;
    }
  } 
  if (withinTol[0]==true && withinTol[1]==true && withinTol[2]==true && withinTol[3]==true && withinTol[4]==true && withinTol[5]==true){
    Serial.println("Power off position achieved");
  }
}

void singlePowerOff(int x){
  Serial.println("Moving to power off position");
  float powerOffpos[6] = {0, 132, 141, 0, 0, 0};
  bool withinTol[6] = {false, false, false, false, false, false};
  steppers[x].moveTo(powerOffpos[x]);
  steppers[x].run();
  if(abs(encoders[x].read()-powerOffpos[x])<encTol*2) {
    Serial.println("Power off position achieved");
  } 
}

void motorInputAngle(int x){
  bool targetReached = false;

  Serial.println("This is the input angle function. Enter an angle and then the motor will move to that angle.");
  Serial.println("The program will stay in this function until you type 'Back', which will take you to the SINGLE MOTOR MENU.");
  
  //Read serial input and trim the length of value
  if (calibrated[x] != true) {
    Serial.println("Motor not calibrated. Calibrating now...");
    singleCalibrate(x);
  }
  String input;
  clearScreen();
  while(input != "Back"){
    Serial.println();
    Serial.println();

    Serial.println("Please enter desired output angle or enter 'Back' to return to the Single Motor Menu:");
    while (Serial.available() == 0);
    input = Serial.readStringUntil('\n');
    input.trim();

    if(input != "Back") {
      //Used for validate trajectory function
      float deg = input.toFloat();
          
          
      //If path = 0, take counterclockwise path, if path = 1, take clockwise path, if path = 2 Don't move
      int path = ValidateTraj(x,deg);
      if (path == 2) {
        continue;
      }
      //If given an input...
      
        //step position motor must turn to
        long targetSteps = (long)(deg*stepsDeg[x]);

        Serial.print("Moving to ");
        Serial.print(deg);
        Serial.print(" degrees or ");
        Serial.print(deg*stepsDeg[x]);
        Serial.println(" steps");

        //Use encoder proportional control to move to target
        encoderRunToVal(x, targetSteps, path);

        if(abs(encoders[x].read()/countstep[x]/stepsDeg[x]-deg)<encTol*2) {
          targetReached = true;
          Serial.println("Target Reached");
        }
        singleDispEnc(x);
      
    }
  }
  clearScreen();
}

//NEEDS TESTING
//MAY HAVE TO CHANGE SO ALL MOTORS RUN AT SAME TIME
void armInputAngle(){
  Serial.println("This is the Arm Input Angle function. Enter an angle, press enter, and repeat 6 times. The function will verify their validity and move to those angles.");
  Serial.println("If you would like to reinput your angles type 'Restart'.");
  Serial.println("The program will stay in this function until you type 'Back', which will take you to the FULL ARM MENU.");
  
  //Read serial input and trim the length of value
  for (int i=0;i<6;i++){
    if (calibrated[i] != true) {
      //fullCalibrate();
      break;
    }
  }
  while (Serial.available() == 0);
  String input1 = Serial.readStringUntil('\n');
  input1.trim();
  String input[6];
  float deg[6];
  int path[6];
  long targetSteps[6];
  bool targetReached[6] = {false, false, false, false, false, false};
  bool allReached = false;
  while(input1 != 'Back'){
    Serial.println("Please enter desired output angles or enter 'Back' to return to the Single Motor Menu:");
    
    for(int i=0;i<6;i++){
      while (Serial.available() == 0);
      input[i] = Serial.readStringUntil('\n');
      input[i].trim();

      //Used for validate trajectory function
      
      deg[i] = input[i].toFloat();
          
          
      //If path = 0, take counterclockwise path, if path = 1, take clockwise path, if path = 2 Don't move
      
      path[i] = ValidateTraj(i,deg[i]);
      if (path[i] == 2) {
        return;
      }
      //If given an input...
      if (input[i].length() > 0){
        //step position motor must turn to
        targetSteps[i] = (long)(deg[i]*stepsDeg[i]);

        Serial.print("Moving to ");
        Serial.print(deg[i]);
        Serial.print(" degrees or ");
        Serial.print(deg[i]*stepsDeg[i]);
        Serial.println(" steps");

        //Use encoder proportional control to move to target
        encoderRunToVal(i, targetSteps[i], path[i]);

        if(abs(encoders[i].read()/countstep[i]/stepsDeg[i]-deg[i])<encTol*2) {
          targetReached[i] = true;
        } 
        if (targetReached[i] != true) {
          allReached = false;
          break;
        }
      }
    }    
    if (allReached){
      Serial.println("Target Reached");
    }
  }
}

// Inverse kinematics function that moves the robot to a certain point
//NOT AVAILABLE ATM
void moveToPoint(){
  Serial.println("This is the Move to Point function. Enter a point as '(x,y,z)' and then the motor will verify its validity and move to that point.");
  Serial.println("The program will stay in this function until you type 'Back', which will take you to the FULL ARM MENU.");
}

//Displays 
void singleDispEnc(int x){
  Serial.println("Encoder value is:");
  Serial.println("Encoder:    Degrees:    Steps:    Counts:");
  Serial.print(x);
  Serial.print("           ");
  Serial.print(encoders[x].read()/countstep[x]/stepsDeg[x]);
  Serial.print("        ");
  Serial.print(encoders[x].read()/countstep[x]);
  Serial.print("      ");
  Serial.println(encoders[x].read());
}

//Displays all the encoder values in a table format
void fullDispEnc(){
  Serial.println("Encoder values are:");
  Serial.println("Encoder:    Degrees:    Steps:    Counts:");
  for (int i=0;i<6;i++){
    Serial.print(i);
    Serial.print("            ");
    Serial.print(encoders[i].read()/countstep[i]/stepsDeg[i]);
    Serial.print("        ");
    Serial.print(encoders[i].read()/countstep[i]);
    Serial.print("       ");
    Serial.println(encoders[i].read());
  }
}

void clearScreen(){
  for (int i=0;i<20;i++){
    Serial.println();
  }
}

void menu(){
  int s1 = 0;
  while (s1 != 3){
    clearScreen();
    Serial.println("      MAIN MENU \n1. Single Motor Movement \n2. Full Arm Movement \nPlease enter the number you desire.");

    while (Serial.available() == 0);
    String selection1 = Serial.readStringUntil('\n');
    selection1.trim();
    s1 = selection1.toInt();

    switch (s1) {
      case 1: {
        clearScreen();
        Serial.println("Enter Motor Number:");
        while (Serial.available() == 0);
        String motorInput = Serial.readStringUntil('\n');
        motorInput.trim();
        int mInput = motorInput.toInt() - 1; 
        clearScreen();
        int s2 = 0;
        while(s2 != 4) {
          Serial.println();
          Serial.println();
          Serial.println("      SINGLE MOTOR MENU \n1. Calibrate \n2. Read Encoder Value \n3. Move to angle \n4. Back to MAIN MENU \nNOTE: If not previously calibrated, option 3 will automatically calibrate before moving to desired angles \nPlease enter the number you desire.");
          
          while (Serial.available() == 0);
          String selection2 = Serial.readStringUntil('\n');
          selection2.trim();
          s2 = selection2.toInt();
          clearScreen();

          switch(s2) {
            case 1:
              Serial.print("Calibrating motor ");
              Serial.print(mInput+1);
              Serial.println("...");
              singleCalibrate(mInput);
              Serial.println("Calibration Complete.");
              break;
            case 2:
              singleDispEnc(mInput);
              break;
            case 3:
              motorInputAngle(mInput);
              break;
            case 4:
              Serial.println("Backing out of SINGLE MOTOR MENU");
              break;
            }
        }
        break;
      }
      case 2: {
        clearScreen();
        //FULL ARM DEVELOPMENT NOT AVAILABLE YET
        int s3 = 0;
        while (s3 != 5){
          Serial.println("      Full ARM MENU \n1. Calibrate \n2. Read Encoder Values \n3. Move to Point(Inverse Kinematics) \n4. Input Angles(Forward Kinematics) \n5. Back to MAIN MENU \nNOTE: If not previously calibrated, option 3 will automatically calibrate before moving to desired angles \nPlease enter the number you desire.");
          while (Serial.available() == 0);
          String selection3 = Serial.readStringUntil('\n');
          selection3.trim();
          s3 = selection3.toInt();
          clearScreen();
          switch(s3) {
            case 1:
            /*
              Serial.print("Calibrating arm...");
              fullCalibrate();
              Serial.println("Calibration Complete.");
              */
              break;
            case 2:
              fullDispEnc();
              break;
            case 3:
              moveToPoint();
              break;
            case 4:
              armInputAngle();
              break;
            case 5:
              Serial.println("Backing out of FULL ARM MENU");
              break;
          }
        }
        break;
      }
      case 3:
        Serial.print("Exiting Now...");
        break;
      
    }
  }
}