// Include the ROME library.
#include <ROME.h>

#define WHEEL_DIAMETER 0.1016  // Diameter in meters
#define PI 3.14159265358979323846  // Pi
#define PULSES_PER_REVOLUTION 6  //motor's specification
#define gearRatio 50 // 50:1 gear reduction with gear box attached

// Constants for hysteresis and debounce
const float hysteresisThreshold = 0.05; // Speed threshold to avoid toggling at zero cross
const unsigned long debounceTime = 150; // Debounce time in milliseconds

unsigned long lastDirectionChangeTime[4] = {0, 0, 0, 0}; // Track last change time for debouncing

// Array to store the last pulse time for each motor
volatile unsigned long lastPulseTime[4] = {0, 0, 0, 0};  // Assuming 4 motors

// Arrays to store the calculated speeds and RPMs
volatile int pulseCount[4] = {0, 0, 0, 0};
volatile float actualLinearSpeeds[4] = {0.0, 0.0, 0.0, 0.0};
volatile float actualRPMs[4] = {0.0, 0.0, 0.0, 0.0};
int motorDirection[4] = {1, 1, 1, 1}; // 1 for forward, -1 for reverse
volatile float angularVelocities[4] = {0.0,0.0,0.0,0.0};
// Proportional Gain for the P controller
const float Kp = 1.0;

// Instantiate four motors with their respective pins.
// Arguments for ROME (PWM pin (yellow), Hall Effect sensor pin (blue), Relay pin (one of 4 colors))
ROME motor1(6,2,9);  
ROME motor2(5,3,10); 
ROME motor3(7,18,11);
ROME motor4(8,19,12);


float u1_values[] = {60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60};
float u2_values[] = {0, 5.514500e-16, 5.164700e-16, 4.814900e-16, 6.453000e-16, 6.765900e-16, 9.729200e-16, 1.269200e-15, 4.313100e-16, 7.276400e-16, 1.024000e-15, 1.320300e-15, 1.616600e-15, 1.234700e-15, 9.268600e-16, 1.075000e-15, 2.283400e-15, 2.579700e-15, -7.874000e-15, -8.711900e-15, -8.489700e-15, -8.267400e-15, -8.045200e-15, -7.897000e-15, -7.748800e-15, 2.927100e-15, 3.001200e-15, 3.149300e-15, 2.089200e-15, 2.311400e-15, 2.533700e-15, 2.533700e-15, 2.755900e-15, 2.830000e-15, 2.904100e-15, 2.904100e-15, 2.978100e-15, 3.052200e-15, 3.163400e-15, 3.274500e-15, 3.385600e-15, 3.496700e-15, 1.487500e-15, 3.719000e-15, 3.793100e-15, 1.746800e-15, 1.857900e-15, 1.969000e-15, 4.237600e-15, 4.311600e-15, 4.348700e-15, 2.265400e-15, 2.339400e-15, 4.533900e-15, 2.450600e-15, 2.598700e-15, 2.746900e-15, 5.015400e-15, 2.932100e-15, 3.006200e-15, 5.200600e-15};
float u4_values[] = {-60, -60, -60, -60, -60, -60, -60, -60, -60, -60, -60, -60, -60, -60, -60, -60, -60, -60, -60, -60, -60, -60, -60, -60, -60, -60, -60, -60, -60, -60, -60, -60, -60, -60, -60, -60, -60, -60, -60, -60, -60, -60, -60, -60, -60, -60, -60, -60, -60, -60, -60, -60, -60, -60, -60, -60, -60, -60, -60, -60, -60, -60};
float u3_values[] = {0, 5.514500e-16, 5.164700e-16, 4.814900e-16, 6.453000e-16, 6.765900e-16, 9.729200e-16, 1.269200e-15, 4.313100e-16, 7.276400e-16, 1.024000e-15, 1.320300e-15, 1.616600e-15, 1.234700e-15, 9.268600e-16, 1.075000e-15, 2.283400e-15, 2.579700e-15, -7.874000e-15, -8.711900e-15, -8.489700e-15, -8.267400e-15, -8.045200e-15, -7.897000e-15, -7.748800e-15, 2.927100e-15, 3.001200e-15, 3.149300e-15, 2.089200e-15, 2.311400e-15, 2.533700e-15, 2.533700e-15, 2.755900e-15, 2.830000e-15, 2.904100e-15, 2.904100e-15, 2.978100e-15, 3.052200e-15, 3.163400e-15, 3.274500e-15, 3.385600e-15, 3.496700e-15, 1.487500e-15, 3.719000e-15, 3.793100e-15, 1.746800e-15, 1.857900e-15, 1.969000e-15, 4.237600e-15, 4.311600e-15, 4.348700e-15, 2.265400e-15, 2.339400e-15, 4.533900e-15, 2.450600e-15, 2.598700e-15, 2.746900e-15, 5.015400e-15, 2.932100e-15, 3.006200e-15, 5.200600e-15};
//float u3_values[] = {60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60};
//float u4_values[] = {60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60};
float timeValues[] = {0, 0.25, 0.5, 0.75, 1.0, 1.25, 1.5, 1.75, 2.0, 2.25, 2.5, 2.75, 3.0, 3.25, 3.5, 3.75, 4.0, 4.25, 4.5, 4.75, 5.0, 5.25, 5.5, 5.75, 6.0, 6.25, 6.5, 6.75, 7.0, 7.25, 7.5, 7.75, 8.0, 8.25, 8.5, 8.75, 9.0, 9.25, 9.5, 9.75, 10.0, 10.25, 10.5, 10.75, 11.0, 11.25, 11.5, 11.75, 12.0, 12.25, 12.5, 12.75, 13.0, 13.25, 13.5, 13.75, 14.0, 14.25, 14.5, 14.75, 15.0};


// Calculate the number of values in one of the arrays.
// It's assumed that all arrays are of the same length.
const int numberOfValues = sizeof(u1_values) / sizeof(u1_values[0]);

// currentIndex keeps track of which set of values (from the arrays) is currently being used to control the motors.
int currentIndex = 0;

// An array that stores the previous direction of each motor. // 1 indicates forward and -1 indicates reverse, 0 off.
int previousDirection[] = {0, 0, 0, 0};

// Create an array of ROME motor objects for easier looping and control.
ROME motors[] = {motor1, motor2, motor3, motor4};

void setup() 
{
  Serial.begin(115200);
  attachInterrupt(digitalPinToInterrupt(2), updateSpeed1, FALLING);
  attachInterrupt(digitalPinToInterrupt(3), updateSpeed2, FALLING);
  attachInterrupt(digitalPinToInterrupt(18), updateSpeed3, FALLING);
  attachInterrupt(digitalPinToInterrupt(19), updateSpeed4, FALLING);
}

// This function runs repeatedly after the setup() finishes.
void loop() {
    if (currentIndex < numberOfValues) {

        float desiredRPMs[] = {u1_values[currentIndex], u2_values[currentIndex], u3_values[currentIndex], u4_values[currentIndex]};
        
        bool* directionChanges = needsDirectionChange(desiredRPMs);

        // Loop through each motor to see if it needs a direction change
        for (int i = 0; i < 4; i++) {
            if (directionChanges[i]) {
                motors[i].disableMotor();
            }
        }
        //delay(0); // This delay is constant and might be necessary for the direction change and reactivation of the motor.
        // Set the direction for each motor based on the speeds array.
        setAllDirections(desiredRPMs);

        for (int i = 0; i < 4; i++) {

          double error = desiredRPMs[i] - actualRPMs[i];
          double deltaSpeed = Kp * error;
          //float newMotorInput = deltaSpeed + actualRPMs[i];
          setMotorSpeed(motors[i],actualRPMs[i] + deltaSpeed);
          // Debugging outputs
          Serial.println("\n Motor ");
          Serial.print(i + 1);
          Serial.print(", RPMs = ");
          Serial.print(actualRPMs[i]);
          Serial.print(": Desired Speed (RPM) = ");
          Serial.print(desiredRPMs[i]);
          Serial.print(": Angular Velocity = ");
          Serial.print(angularVelocities[i]);
          Serial.print(", Linear Velocity = ");
          Serial.print(actualLinearSpeeds[i]);
          Serial.print(", Error = ");
          Serial.print(error);
          Serial.print(", Motor Delta = "); 
          Serial.print(deltaSpeed);
          Serial.print('\n'); 

        }

        currentIndex++;

        // Compute the delay based on the difference between the current time and the previous time
        if (currentIndex < numberOfValues) {
            float timeDifference = timeValues[currentIndex] - timeValues[currentIndex - 1];
            delay(timeDifference*1000); // Assuming timeValues[] is in seconds since delay() function expects milliseconds
        }
    } else {
        motor1.disableMotor();
        motor2.disableMotor();
        motor3.disableMotor();
        motor4.disableMotor();
        
        while(1); // Infinite loop once done
    }
}

void updateSpeed1() { calculateSpeed(0); }
void updateSpeed2() { calculateSpeed(1); }
void updateSpeed3() { calculateSpeed(2); }
void updateSpeed4() { calculateSpeed(3); }


void calculateSpeed(int motorIndex) {
  unsigned long currentTime = millis();
  pulseCount[motorIndex]++; // Increment pulse count on each interrupt call
  
  if (lastPulseTime[motorIndex] != 0) {
    unsigned long timeDifference = currentTime - lastPulseTime[motorIndex];
    double timeInSeconds = timeDifference / 1000.0; // Convert milliseconds to seconds

    // Calculate pulses per minute
    double pulsesPerMinute = (pulseCount[motorIndex] / timeInSeconds) * 60;

    // Calculate RPM
    double rpm = pulsesPerMinute / PULSES_PER_REVOLUTION / gearRatio;
    actualRPMs[motorIndex] = motorDirection[motorIndex] * rpm;

    // Calculate linear speed
    double revolutionsPerSecond = pulseCount[motorIndex] / timeInSeconds / PULSES_PER_REVOLUTION;
    double wheelCircumference = PI * WHEEL_DIAMETER; 
    actualLinearSpeeds[motorIndex] = motorDirection[motorIndex] * revolutionsPerSecond * wheelCircumference;

    // Calculate angular velocity
    angularVelocities[motorIndex] = motorDirection[motorIndex] * (2 * PI * rpm / 60);

    // Reset pulse count after calculations
    pulseCount[motorIndex] = 0;
  }
  lastPulseTime[motorIndex] = currentTime;  // Update the last pulse time for future calculations
}

// void calculateSpeed(int motorIndex) {
//   unsigned long currentTime = millis();
//   if (lastPulseTime[motorIndex] != 0) {
//     unsigned long timeDifference = currentTime - lastPulseTime[motorIndex];
//     float timeInSeconds = timeDifference / 1000.0; // Convert milliseconds to seconds

//     // Calculate pulses per second
//     float pulsesPerSecond = 1 / timeInSeconds;
    
//     // Calculate pulses per minute
//     float pulsesPerMinute = pulsesPerSecond * 60;

//     // Calculate RPM by dividing pulses per minute by pulses per revolution
//     float rpm = (pulsesPerMinute / PULSES_PER_REVOLUTION)/gearRatio;

//     // Calculate linear speed using the circumference of the wheel, adjusted for direction
//     float revolutionsPerSecond = pulsesPerSecond / PULSES_PER_REVOLUTION;
//     float wheelCircumference = PI * WHEEL_DIAMETER; 
//     actualLinearSpeeds[motorIndex] = motorDirection[motorIndex] * revolutionsPerSecond * wheelCircumference;

//     // Calculate angular velocity
//     angularVelocities[motorIndex] = 2 * PI * rpm / 60;
//   }
//   lastPulseTime[motorIndex] = currentTime;  // Update the last pulse time
// }

// void calculateSpeed(int motorIndex) {
//   unsigned long currentTime = millis();
//   if (lastPulseTime[motorIndex] != 0) {
//     unsigned long timeDifference = currentTime - lastPulseTime[motorIndex];
//     float timeInSeconds = timeDifference / 1000.0; // Convert milliseconds to seconds

//     // Calculating RPM based on direction
//     float revolutionsPerSecond = (1 / timeInSeconds) / PULSES_PER_REVOLUTION;
//     actualRPMs[motorIndex] = motorDirection[motorIndex] * revolutionsPerSecond * 60/(50);

//     // Calculating linear speed using the circumference of the wheel, adjusted for direction
//     float wheelCircumference = PI * WHEEL_DIAMETER; 
//     actualLinearSpeeds[motorIndex] = motorDirection[motorIndex] * revolutionsPerSecond * wheelCircumference;

//     // Calculate angular velocity
//     angularVelocities[motorIndex] = 2 * PI * actualRPMs[motorIndex] / 60;
//   }
//   lastPulseTime[motorIndex] = currentTime;  // Update the last pulse time
// }



// Given a motor, its previous direction, and the current speed value, 
// this function sets the direction of the motor.

void setMotorDirection(ROME &motor, int &previousDir, float currentSpeedValue, int motorIndex) {
    unsigned long currentTime = millis();
    int currentDir = (currentSpeedValue > hysteresisThreshold) ? 1 : (currentSpeedValue < -hysteresisThreshold) ? -1 : previousDir;

    if (previousDir != currentDir && (currentTime - lastDirectionChangeTime[motorIndex] > debounceTime)) {
        motor.disableMotor();
       // delay(10); // Short delay to physically allow the motor to stop

        if (currentDir > 0) {
            motor.motorForward();
        } else if (currentDir < 0) {
            motor.motorReverse();
        }

        previousDir = currentDir;
        motorDirection[motorIndex] = currentDir; // Update the global direction
        lastDirectionChangeTime[motorIndex] = currentTime;
    }
}
// void setMotorDirection(ROME &motor, int &previousDir, float currentSpeedValue, int motorIndex) {
//     // Determine the current direction based on the sign of the speed value.
//     int currentDir = (currentSpeedValue >= 0) ? 1 : -1; 
    
//     // If the direction has changed, stop the motor briefly.
//     if (previousDir != currentDir) {
//         motor.disableMotor();
//         // Optionally add a delay if needed
//     }

//     // Set the motor's direction based on the determined current direction.
//     if (currentDir > 0) {
//         motor.motorForward();
//     } else {
//         motor.motorReverse();
//     }
    
//     // Update the stored previous direction for the next check.
//     previousDir = currentDir;
//     motorDirection[motorIndex] = currentDir; // Update the motor direction globally
// }

// Given a motor and a speed value, this function sets the speed of the motor.
void setMotorSpeed(ROME &motor, float currentSpeedValue) {

    // Map the given speed value to an appropriate range for the motor.
    float currentSpeed = map(currentSpeedValue, -115, 115, -255, 255);
    
    // Activate the motor with the mapped speed value.
    motor.motorOn(abs(currentSpeedValue));
}

// function sets the direction of all motors based on the given speeds array.
void setAllDirections(float speeds[]) {
    for (int i = 0; i < 4; i++) {
        setMotorDirection(motors[i], previousDirection[i], speeds[i], i);
    }
}


// function checks if any motor needs to change its direction based on the provided speeds.
bool* needsDirectionChange(float speeds[]) {
    static bool directionChange[4];
    for (int i = 0; i < 4; i++) {
        int currentDir = (speeds[i] >= 0) ? 1 : -1;
        if (previousDirection[i] != currentDir) {
            directionChange[i] = true;
        } else {
            directionChange[i] = false;
        }
    }
    return directionChange;
}