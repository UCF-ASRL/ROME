#include <AccelStepper.h>

// Define interface type for step/direction driver
#define DRIVER_INTERFACE AccelStepper::DRIVER

// Pin assignments for DM542T
#define STEP_PIN 10  // PUL
#define DIR_PIN 11   // DIR

// Create stepper motor instance
AccelStepper stepper(DRIVER_INTERFACE, STEP_PIN, DIR_PIN);

void setup() {
  // Set speed and acceleration (adjust as needed)
  stepper.setMaxSpeed(1000);     // steps per second
  stepper.setAcceleration(500);  // steps per second^2

  // Initial movement target
  stepper.moveTo(2000);
}

void loop() {
  // If at target, reverse direction
  if (stepper.distanceToGo() == 0) {
    stepper.moveTo(-stepper.currentPosition());
  }

  // Step motor towards target
  stepper.run();
}
