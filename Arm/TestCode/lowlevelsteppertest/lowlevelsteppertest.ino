//#include <imxrt.h>
#define STEP_PIN 6
#define DIR_PIN 7

const int stepsPerHalfRev = 1200;   // 180 degrees at full-step
const int stepDelayMicros = 10;  // Adjust for speed (1000 µs = 1 kHz)
const int stepIntervalMicros = 1200;
void setup() {
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  //Set drive strength for stepper 4
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_10 =
    IOMUXC_PAD_DSE(2) |
    IOMUXC_PAD_SPEED(2) |
    IOMUXC_PAD_SRE; 
  IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_11 =
    IOMUXC_PAD_DSE(2) |
    IOMUXC_PAD_SPEED(2) |
    IOMUXC_PAD_SRE;
  digitalWrite(6, HIGH);
  digitalWrite(7, HIGH);
  
}

void loop() {
  // Rotate 180 degrees clockwise
  digitalWrite(DIR_PIN, HIGH);
  stepMotor(stepsPerHalfRev);
  Serial.println("Turning to 180");
  delay(1000);  // Wait 1 second

  // Rotate 180 degrees counter-clockwise
  digitalWrite(DIR_PIN, LOW);
  stepMotor(stepsPerHalfRev);
  Serial.println("Turning to 0");
  delay(1000);  // Wait 1 second
}

void stepMotor(int steps) {
  for (int i = 0; i < steps; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(stepDelayMicros);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(stepIntervalMicros - stepDelayMicros);
  }
}
