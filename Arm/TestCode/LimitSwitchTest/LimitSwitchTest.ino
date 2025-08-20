class LimitSwitch{
  public:
    int pin;
    //state of the switch. 0 = open, 1 = closed
    int state;

};
//Time for debouncing limit switches
const unsigned long debounceTime = 100;
unsigned long LastDebounce[6] = {};
LimitSwitch LS[6];
int pn = 0;

void setup() {
  Serial.begin(19200);
  //Limit switch setup
    for (int i = 0; i<6; i++) {
      //Signal pins on teensy
      LS[i].pin = 26+i;
      //Pulls signal pin up
      pinMode(LS[i].pin, INPUT_PULLUP);
      
  }

}

void loop() {
  // put your main code here, to run repeatedly:
  

  if (digitalRead(LS[0].pin) == LOW) {
    if(millis() - LastDebounce[0] > debounceTime){
      Serial.println("1 - LOW - PRESSED");
      LastDebounce[0] = millis();
    }
  }
  if (digitalRead(LS[1].pin) == LOW) {
    if(millis() - LastDebounce[1] > debounceTime){
      Serial.println("2 - LOW - PRESSED");
      LastDebounce[1] = millis();
    }
  }
  if (digitalRead(LS[2].pin) == LOW) {
    if(millis() - LastDebounce[2] > debounceTime){
      Serial.println("3 - LOW - PRESSED");
      LastDebounce[2] = millis();
    }
  }
  if (digitalRead(LS[3].pin) == LOW) {
    if(millis() - LastDebounce[3] > debounceTime){
      Serial.println("4 - LOW - PRESSED");
      LastDebounce[3] = millis();
    }
  }
  if (digitalRead(LS[4].pin) == LOW) {
    if(millis() - LastDebounce[4] > debounceTime){
      Serial.println("5 - LOW - PRESSED");
      LastDebounce[4] = millis();
    }
  }
  if (digitalRead(LS[5].pin) == LOW) {
    if(millis() - LastDebounce[5] > debounceTime){
      Serial.println("6 - LOW - PRESSED");
      LastDebounce[5] = millis();
    }
  }
}
