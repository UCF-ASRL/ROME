#include <AccelStepper.h>

AccelStepper stepper4(AccelStepper::DRIVER, 2, 3);

void setup() {
  Serial.begin(9600); 

  // Force pin 6 and 7 to be GPIO
  IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_10 = 5;
  IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_11 = 5;
  
  //pinMode(6, OUTPUT);
  //pinMode(7, OUTPUT);
  
  //digitalWrite(6, HIGH);
  //digitalWrite(7, HIGH);
  stepper4.setMaxSpeed(1000);
  stepper4.setAcceleration(500);
  stepper4.setMinPulseWidth(10);
  //stepper4.setEnablePin(12); // optional
  stepper4.enableOutputs();
  
  IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_10 =
    IOMUXC_PAD_DSE(3) |
    IOMUXC_PAD_SPEED(2) |
    IOMUXC_PAD_SRE; 

  IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_11 =
    IOMUXC_PAD_DSE(3) |
    IOMUXC_PAD_SPEED(2) |
    IOMUXC_PAD_SRE;

  digitalWrite(6, HIGH);
  digitalWrite(7, HIGH);
  stepper4.moveTo(1000);
}

void loop() {
  stepper4.run();
  if (stepper4.distanceToGo() == 0) {
    // Toggle destination once target reached
    if (stepper4.currentPosition() == 1000) {
      stepper4.moveTo(0);
    } else {
      stepper4.moveTo(1000);
    }
  }
}
