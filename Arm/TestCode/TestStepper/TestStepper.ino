#include <AccelStepper.h>
#include <Encoder.h>
// Define interface type for step/direction driver
#define DRIVER_INTERFACE AccelStepper::DRIVER

// Pin assignments for DM542T
#define STEP_PIN 0  // PUL
#define DIR_PIN 1   // DIR
float countstep = 20.48; //counts/step
int x = 0;
Encoder encoders[] = {
  Encoder(14,15),
  Encoder(17,16),//reversed because encoder direction does not align with motor direction
  Encoder(19,18),
  Encoder(20,21),
  Encoder(22,23),
  Encoder(24,25)
};

// Create stepper motor instance
AccelStepper stepper(DRIVER_INTERFACE, STEP_PIN, DIR_PIN);

void setup() {
  delay(10000);
  // Set speed and acceleration (adjust as needed)
  stepper.setMaxSpeed(500);     // steps per second
  stepper.setAcceleration(500);  // steps per second^2
  stepper.setMinPulseWidth(5);
  encoders[x].write(0);
  stepper.setCurrentPosition(0);
  // Initial movement target
  stepper.moveTo(1000);
}
/*
void loop() {
  // If at target, reverse direction
  if (stepper.distanceToGo() == 0) {
    Serial.print("Target Reached. Position is: ");
    Serial.println(encoders[x].read());
    delay(15);
    Serial.println("New Target");
    stepper.moveTo(-stepper.currentPosition());
  }

  // Step motor towards target
  stepper.run();
}
*/

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.length() > 0){
      float targetsteps = input.toFloat();
      

      Serial.print("Moving to ");
      Serial.print(targetsteps);
      Serial.println(" steps");

      while(true) {
        int encoderval = encoders[x].read();
        Serial.println(encoderval);
        Serial.println(encoderval/countstep);
        long actual = encoderval/countstep;
        long error = targetsteps - actual;
        
        if(abs(error) < 10){
          break;
        }
        stepper.setSpeed(500);
        stepper.moveTo(stepper.currentPosition() + error);
        stepper.run();
      }
      

      
/*
      while(steppers[x].distanceToGo() != 0) {
        steppers[x].run();
      }
*/

      Serial.println("Position reached.");
      float pos = encoders[x].read();
      Serial.print("Encoder Position is:");
      Serial.print(pos*countstep);
      Serial.print(" steps Or ");
    }
  }
}
