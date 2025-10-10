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
int x = 4;
PacketSerial packetizer;

//Initialize encoder object
Encoder encoders[] = {
  Encoder(14,15),
  Encoder(16,17),
  Encoder(18,19),
  Encoder(20,21),
  Encoder(22,23),
  Encoder(24,25)
};

// Expressed in Joint frame
long steps[6] = {0,0,0,0,0,0}; 
float stepsSec[6] = {0.0,0.0,0.0,0.0,0.0,0.0};

//Ar2 motor values
//Mech transformation from output theta to motor input
float stepsDeg[6] = {232.48525, 258.17777, 281.91111, 0, 1.8, 0};
//44.70833342 232.48525
float limits[6] = {-180, -132, 1, -164, 104, 148};

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
  //calibrate();
  steppers[x].setCurrentPosition(0);
  encoders[x].write(0);
  
  steppers[x].setSpeed(500);
}
void loop() {
steppers[x].moveTo(5000);
steppers[x].run();
Serial.println(encoders[x].read());
}
